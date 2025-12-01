"""This file acts as the main module for this script."""

import traceback
import adsk.core
import adsk.fusion
import os
import json
import math


# =========================
#  Core data structures
# =========================

class Point2D:
    def __init__(self, u, v, x=None, y=None):
        # Local coordinates (given by input JSON)
        self.u = float(u)
        self.v = float(v)

        # Global coordinates (computed later by the position problem)
        self.x = None if x is None else float(x)
        self.y = None if y is None else float(y)

    @staticmethod
    def from_json(raw_point):
        # raw_point: [u, v] in local link coordinates
        return Point2D(raw_point[0], raw_point[1])

    def set_global(self, x, y):
        """Set global coordinates after solving the position problem."""
        self.x = float(x)
        self.y = float(y)

class Link:
    def __init__(self, link_id, ground=False, plane=0, pts=None):
        self.id = link_id
        self.ground = bool(ground)
        self.plane = int(plane)
        self.pts = pts if pts is not None else {}
        # Filled during generate()
        self.component = None
        self.occurrence = None
        self.body = None

    @staticmethod
    def from_json(link_id, data):
        """
        data:
          {
            "ground": true/false,
            "plane": 0,
            "pts": { "A": [u, v], ... }
          }
        """
        pts = {}
        pts_raw = data.get("pts", {})
        for name, coords in pts_raw.items():
            pts[name] = Point2D.from_json(coords)

        return Link(
            link_id,
            ground=data.get("ground", False),
            plane=data.get("plane", 0),
            pts=pts
        )

    def generate(self):
        global root_comp  # use global design component

        # 1) Collect points with global coordinates
        global_pts = [p for p in self.pts.values() if p.x is not None and p.y is not None]
        if len(global_pts) == 0:
            return

        pts_xy = [(p.x, p.y) for p in global_pts]

        # 2) Radii and thickness (tune as you like)
        link_radius    = 0.5   # outer pad radius
        hole_radius    = 0.25  # joint hole
        link_thickness = 0.5   # extrusion thickness AND plane spacing

        # 3) Create a new component for this link
        occs = root_comp.occurrences
        transform = adsk.core.Matrix3D.create()
        occ = occs.addNewComponent(transform)
        comp = occ.component
        self.component = comp   # store for later (joints)
        self.occurrence = occ        # ⬅️ important: used by connect()
        self.body = None        # will fill in after extrude

        # 4) Choose sketch plane based on link.plane index, in THIS component
        base_plane  = comp.xYConstructionPlane
        plane_index = getattr(self, "plane", 0)

        if plane_index == 0:
            sketch_plane = base_plane
        else:
            planes = comp.constructionPlanes
            offset_val = adsk.core.ValueInput.createByReal(plane_index * link_thickness)
            p_input = planes.createInput()
            p_input.setByOffset(base_plane, offset_val)
            sketch_plane = planes.add(p_input)

        # 5) Create sketch on that plane
        sketch = comp.sketches.add(sketch_plane)

        # 6) Circles at joints (outer stored for outline building)
        outer_circles = create_joint_circles(sketch, pts_xy, link_radius, hole_radius)

        # 7) Outer outline (if only one joint, just pad+hole, no outline)
        if len(pts_xy) > 1:
            build_link_outline(sketch, pts_xy, link_radius, outer_circles)

        # 8) Extrude outer loop minus inner circles INSIDE THIS COMPONENT
        body = extrude_largest_profile(comp, sketch, link_thickness)
        self.body = body

class Joint:
    """
    Joint connecting:
      (link_i, pt_i_name) <-> (link_j, pt_j_name)
    """
    def __init__(self, joint_id, link_i, pt_i_name, link_j, pt_j_name):
        self.id = joint_id
        self.link_i = link_i
        self.pt_i_name = pt_i_name
        self.link_j = link_j
        self.pt_j_name = pt_j_name

    @property
    def pt_i(self):
        return self.link_i.pts[self.pt_i_name]

    @property
    def pt_j(self):
        return self.link_j.pts[self.pt_j_name]

    @staticmethod
    def from_json(joint_id, data, links):
        """
        data: [
          [linkID_i, pointName_i],
          [linkID_j, pointName_j]
        ]
        links: dict[str, Link]
        """
        (link_i_id, pt_i_name), (link_j_id, pt_j_name) = data

        link_i = links[link_i_id]
        link_j = links[link_j_id]

        return Joint(joint_id, link_i, pt_i_name, link_j, pt_j_name)

class Dyad:
    def __init__(self, dyad_id, links, internal_joint, external_joints, solution=1):
        self.id = dyad_id
        self.links = links              # list[Link]
        self.internal = internal_joint  # Joint
        self.external = external_joints # list[Joint]
        self.solution = solution        # +1 or -1 for circle–circle intersection

    @staticmethod
    def from_json(dyad_id, data, links, joints):
        """
        data:
          {
            "links": ["L1", "L2"],
            "internal": "J1",
            "external": ["J0", ...]
          }
        """
        link_objs = [links[lid] for lid in data.get("links", [])]
        internal_joint = joints[data["internal"]]
        external_joints = [joints[jid] for jid in data.get("external", [])]
        solution = data["solution"] if "solution" in data else 1

        return Dyad(dyad_id, link_objs, internal_joint, external_joints, solution)
    
    def solve_position(self, theta_crank=None):
        """
        Driving dyad:
        - one ground + one moving link
        - we rotate the crank about the internal joint pivot

        Ordinary dyad:
        - internal joint + two external joints
        - propagate external joint positions
        - solve internal joint position
        - solve link poses
        """

        ground_links = [lk for lk in self.links if lk.ground]
        moving_links = [lk for lk in self.links if not lk.ground]

        is_driving = (len(ground_links) == 1) and (len(moving_links) == 1)

        # ==========================================================
        #  DRIVING DYAD  (GROUND + CRANK)
        # ==========================================================
        if is_driving:

            if theta_crank is None:
                raise RuntimeError("Driving dyad requires theta_crank.")

            ground_link = ground_links[0]
            crank_link  = moving_links[0]
            joint       = self.internal

            # internal joint must use same point label on both links
            name_i = joint.pt_i_name
            name_j = joint.pt_j_name
            if name_i != name_j:
                raise RuntimeError(
                    f"Joint {joint.id}: mismatched point labels {name_i} vs {name_j}"
                )
            pivot_name = name_i

            # 1) Ground link points: local == global
            for p in ground_link.pts.values():
                p.set_global(p.u, p.v)

            # Global pivot on ground
            p_g = ground_link.pts[pivot_name]
            xg, yg = p_g.x, p_g.y

            # Local pivot on crank
            p_c = crank_link.pts[pivot_name]
            uc, vc = p_c.u, p_c.v

            # 2) Rotate crank about pivot
            th = float(theta_crank)
            c = math.cos(th)
            s = math.sin(th)

            for p in crank_link.pts.values():
                du = p.u - uc
                dv = p.v - vc
                x = xg + c*du - s*dv
                y = yg + s*du + c*dv
                p.set_global(x, y)

        # ==========================================================
        #  ORDINARY DYAD 
        # ==========================================================
        else:
            # Must have exactly 2 external joints
            if len(self.external) != 2:
                return

            ext1 = self.external[0]
            ext2 = self.external[1]
            Jint = self.internal

            # Internal joint points on the two links
            int_L1 = Jint.pt_i
            int_L2 = Jint.pt_j
            L1     = Jint.link_i
            L2     = Jint.link_j

            # ------------------------------------------------------
            # Step 1 — Propagate external joint coords to dyad links
            # ------------------------------------------------------
            for J in self.external:
                pi = J.pt_i
                pj = J.pt_j
                if pi.x is not None and pj.x is None:
                    pj.set_global(pi.x, pi.y)
                elif pj.x is not None and pi.x is None:
                    pi.set_global(pj.x, pj.y)

            # Helper to find, for a link, which external joint belongs to it
            def ext_point_on(link, ext):
                if ext.link_i is link:
                    return ext.pt_i
                if ext.link_j is link:
                    return ext.pt_j
                return None

            E1 = ext_point_on(L1, ext1) or ext_point_on(L1, ext2)
            E2 = ext_point_on(L2, ext1) or ext_point_on(L2, ext2)
            if E1 is None or E2 is None:
                return
            if E1.x is None or E2.x is None:
                return

            x1, y1 = E1.x, E1.y
            x2, y2 = E2.x, E2.y

            # ------------------------------------------------------
            # Step 2 — Solve internal joint location via circle–circle
            # ------------------------------------------------------
            r1 = math.hypot(int_L1.u - E1.u, int_L1.v - E1.v)
            r2 = math.hypot(int_L2.u - E2.u, int_L2.v - E2.v)

            sol = circle_circle_intersection(x1, y1, r1, x2, y2, r2, solution=self.solution)
            if sol is None:
                return

            xI, yI = sol
            int_L1.set_global(xI, yI)
            int_L2.set_global(xI, yI)

            # ------------------------------------------------------
            # Step 3 — Solve the poses of the two dyad links
            # ------------------------------------------------------
            solve_link_pose_from_two_points(L1, E1, int_L1)
            solve_link_pose_from_two_points(L2, E2, int_L2)
            ground_links = [lk for lk in self.links if lk.ground]
            moving_links = [lk for lk in self.links if not lk.ground]

            is_driving = (len(ground_links) == 1) and (len(moving_links) == 1)

            if is_driving:
                # Driving dyad special case:
                # - Exactly one ground link (inertial frame)
                # - One moving link (crank)
                # - Known crank angle theta_crank
                # - Internal joint is the pivot between them
                if theta_crank is None:
                    raise RuntimeError("Driving dyad requires theta_crank to solve position.")
                
                ground_link = ground_links[0]
                crank_link = moving_links[0]
                joint = self.internal

                # By your convention, the joint always connects
                # the SAME point name on both links.
                name_i = joint.pt_i_name
                name_j = joint.pt_j_name
                if name_i != name_j:
                    raise RuntimeError(
                        f"Joint {joint.id} does not have matching point labels: "
                        f'{name_i} vs {name_j}'
                    )
                pivot_name = name_i  # same on both links

                # 1) Ground link is the inertial frame:
                #    local coordinates == global coordinates.
                for p in ground_link.pts.values():
                    p.set_global(p.u, p.v)

                # Global coordinates of the joint/pivot on the ground link
                p_g = ground_link.pts[pivot_name]
                xg, yg = p_g.x, p_g.y   # typically (0, 0) in your example

                # Local coordinates of that same logical point on the crank link
                p_c = crank_link.pts[pivot_name]
                uc, vc = p_c.u, p_c.v

                # 2) Rotate the crank link about that pivot by theta_crank.
                theta = float(theta_crank)
                c = math.cos(theta)
                s = math.sin(theta)

                # For any point p on the crank:
                #   local delta from pivot: (u - uc, v - vc)
                #   rotate that delta, then add pivot global (xg, yg):
                #   [x; y] = [xg; yg] + R(theta) * [u - uc; v - vc]
                for p in crank_link.pts.values():
                    du = p.u - uc
                    dv = p.v - vc
                    x = xg + c * du - s * dv
                    y = yg + s * du + c * dv
                    p.set_global(x, y)

class Crank:
    def __init__(self, link, joint, theta0):
        self.link = link    # Link
        self.joint = joint  # Joint
        self.theta0 = float(theta0)

    @staticmethod
    def from_json(data, links, joints):
        """
        data:
          {
            "link": "L1",
            "joint": "J0",
            "theta0": 0.0
          }
        """
        if data is None:
            return None
        link = links[data["link"]]
        joint = joints[data["joint"]]
        theta0 = data.get("theta0", 0.0)
        return Crank(link, joint, theta0)

class Mechanism:
    def __init__(self, links, joints, dyads, crank=None):
        self.links = links        # dict[str : Link]
        self.joints = joints      # dict[str : Joint]
        self.dyads = dyads        # list[Dyad]   <--- ordered list
        self.crank = crank        # Crank or None

    @classmethod
    def from_json(cls, raw):
        raw_links = raw.get("LINKS", {})
        raw_joints = raw.get("JOINTS", {})
        raw_dyads = raw.get("DYADS", {})
        raw_crank = raw.get("CRANK", None)

        # 1. Links
        links = {
            lid: Link.from_json(lid, ldata) for lid, ldata in raw_links.items()
        }

        # 2. Joints
        joints = {
            jid: Joint.from_json(jid, jdata, links) for jid, jdata in raw_joints.items()
        }

        # 3. Dyads (ordered list!)
        dyads = []
        for did, ddata in raw_dyads.items():   # preserves JSON order
            dyad = Dyad.from_json(did, ddata, links, joints)
            dyads.append(dyad)

        # 4. Crank
        crank = Crank.from_json(raw_crank, links, joints)

        # return assembled mechanism
        return cls(links, joints, dyads, crank)

    def postion(self, theta_crank):
        """
        Solve the position problem dyad-by-dyad.

        For each dyad:
          - If it is the driving dyad (one ground link, one crank link),
            solve it as a special case: pure rotation of the crank link.
          - Otherwise, invoke the ordinary dyad position solver
            (to be implemented) which uses external joints.
        """
        for i, dyad in enumerate(self.dyads):
            if i == 0:
                # Driving dyad: use given crank angle
                dyad.solve_position(theta_crank=theta_crank)
            else:
                # Ordinary dyads: cascade P-problem (stub for now)
                dyad.solve_position()

    def generate(self):
        """
        Generate the mechanism in Fusion 360.

        This is a stub for now.
        """
        for link in self.links.values():   
            link.generate()

    def connect(self):
        """
        Create Fusion 360 *as-built* revolute joints between link components,
        using the inner joint holes specified by the JSON (link, point name).
        """
        global root_comp

        as_built_joints = root_comp.asBuiltJoints

        # 1) Ground the ground links
        for link in self.links.values():
            occ = getattr(link, "occurrence", None)
            if link.ground and occ is not None:
                occ.isGrounded = True

        # 2) For each logical joint in the JSON, create an as-built revolute joint
        for joint in self.joints.values():
            link_i = joint.link_i
            link_j = joint.link_j

            occ_i = getattr(link_i, "occurrence", None)
            occ_j = getattr(link_j, "occurrence", None)

            if occ_i is None or occ_j is None:
                continue

            # Use the joint's point on each link
            edge_i = find_joint_hole_edge(link_i, joint.pt_i)
            edge_j = find_joint_hole_edge(link_j, joint.pt_j)

            # Prefer an edge from link_j if available; otherwise use link_i
            edge_axis = edge_j or edge_i
            if edge_axis is None:
                continue

            geo = adsk.fusion.JointGeometry.createByCurve(
                edge_axis,
                adsk.fusion.JointKeyPointTypes.CenterKeyPoint
            )

            # As-built joint between the two occurrences, with that geometry as axis
            ab_input = as_built_joints.createInput(occ_i, occ_j, geo)
            ab_input.setAsRevoluteJointMotion(
                adsk.fusion.JointDirections.ZAxisJointDirection
            )

            j = as_built_joints.add(ab_input)
            j.name = joint.id

#======================== Helpers and Globals ========================#
def centroid(points):
    """points: list[(x,y)] → returns (cx, cy)"""
    cx = sum(x for x, y in points) / len(points)
    cy = sum(y for x, y in points) / len(points)
    return cx, cy

def angle_sort(points):
    """Sort points around their centroid by polar angle."""
    cx, cy = centroid(points)
    return sorted(points, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

def outer_tangent_segment(p1, p2, cx, cy, radius):
    """
    Given two circle centers p1=(x1,y1), p2=(x2,y2) and a centroid (cx,cy),
    return the two tangent points that form the outer link boundary.
    """
    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1
    L = math.hypot(dx, dy)
    if L == 0:
        return None  # degenerate

    # Unit tangent direction
    tx = dx / L
    ty = dy / L

    # Normal vector (90° CCW)
    nx0 = -ty
    ny0 =  tx

    # Determine outward direction using centroid
    mx = (x1 + x2) * 0.5
    my = (y1 + y2) * 0.5
    vx = cx - mx
    vy = cy - my

    # If normal points inward, flip it
    if vx * nx0 + vy * ny0 > 0:
        nx = -nx0
        ny = -ny0
    else:
        nx = nx0
        ny = ny0

    # Tangent endpoints
    t1 = (x1 + nx * radius, y1 + ny * radius)
    t2 = (x2 + nx * radius, y2 + ny * radius)
    return t1, t2

def create_joint_circles(sketch, pts_xy, link_radius, hole_radius):
    """
    Draw outer + inner circles at joint centers.
    Returns dict[(x,y)] -> outer SketchCircle.
    """
    circles = sketch.sketchCurves.sketchCircles
    outer_circles = {}
    for (x, y) in pts_xy:
        center = adsk.core.Point3D.create(x, y, 0)
        outer = circles.addByCenterRadius(center, link_radius)
        outer_circles[(x, y)] = outer
        circles.addByCenterRadius(center, hole_radius)
    return outer_circles

def build_link_outline(sketch, pts_xy, link_radius, outer_circles):
    """
    Uses angle_sort + outer_tangent_segment to:
      - draw straight edges between tangent points
      - convert full outer circles to construction
      - add only the outer arcs at each joint
    """
    lines   = sketch.sketchCurves.sketchLines
    arcs    = sketch.sketchCurves.sketchArcs
    spoints = sketch.sketchPoints

    pts_sorted = angle_sort(pts_xy)
    cx, cy = centroid(pts_sorted)
    n = len(pts_sorted)

    # Collect tangent points per joint
    tangents_by_center = {p: [] for p in pts_sorted}

    # Straight edges between tangent points
    for i in range(n):
        p1 = pts_sorted[i]
        p2 = pts_sorted[(i + 1) % n]

        seg = outer_tangent_segment(p1, p2, cx, cy, link_radius)
        if seg is None:
            continue

        (t1x, t1y), (t2x, t2y) = seg

        tangents_by_center[p1].append((t1x, t1y))
        tangents_by_center[p2].append((t2x, t2y))

        sp1 = spoints.add(adsk.core.Point3D.create(t1x, t1y, 0))
        sp2 = spoints.add(adsk.core.Point3D.create(t2x, t2y, 0))
        lines.addByTwoPoints(sp1, sp2)

    # Replace full outer circles with only the outer arcs
    for (cx_j, cy_j), tlist in tangents_by_center.items():
        if len(tlist) < 2:
            continue

        # Make full circle construction
        circ = outer_circles.get((cx_j, cy_j))
        if circ is not None:
            circ.isConstruction = True

        # First two tangency points
        (tAx, tAy), (tBx, tBy) = tlist[0], tlist[1]

        center_pt = adsk.core.Point3D.create(cx_j, cy_j, 0)
        start_pt  = adsk.core.Point3D.create(tAx, tAy, 0)
        end_pt    = adsk.core.Point3D.create(tBx, tBy, 0)

        a0 = math.atan2(tAy - cy_j, tAx - cx_j)
        a1 = math.atan2(tBy - cy_j, tBx - cx_j)

        da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))

        if da > 0:
            sweep_small = da
            sweep_big = da - 2.0 * math.pi
        else:
            sweep_small = da
            sweep_big = da + 2.0 * math.pi

        vx_cent = cx - cx_j
        vy_cent = cy - cy_j

        def is_outer(sweep):
            am = a0 + sweep * 0.5
            rm_x = math.cos(am)
            rm_y = math.sin(am)
            return (rm_x * vx_cent + rm_y * vy_cent) < 0.0

        sweep = sweep_big if is_outer(sweep_big) else sweep_small

        arcs.addByCenterStartSweep(center_pt, start_pt, sweep)

def extrude_largest_profile(component, sketch, thickness):
    """
    Find the largest profile in the sketch and extrude it
    as a new body in the given component.
    """
    if sketch.profiles.count == 0:
        return None

    largest_profile = None
    max_area = 0.0
    for prof in sketch.profiles:
        try:
            area = prof.areaProperties().area
        except:
            area = 0.0
        if area > max_area:
            max_area = area
            largest_profile = prof

    if largest_profile is None:
        return None

    extrudes = component.features.extrudeFeatures
    distance = adsk.core.ValueInput.createByReal(thickness)

    ext_input = extrudes.createInput(
        largest_profile,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    ext_input.setDistanceExtent(False, distance)

    ext = extrudes.add(ext_input)

    # Return the created body
    if ext.bodies.count > 0:
        return ext.bodies.item(0)
    return None

def find_joint_hole_edge(link, pt, tol=1e-3):
    """
    link : Link (with link.body and link.occurrence set by generate())
    pt   : Point2D with global x, y
    tol  : 2D distance tolerance in sketch units

    Returns the *assembly-context* circular edge corresponding
    to the inner joint hole of this link at point pt.
    """
    body = getattr(link, "body", None)
    occ  = getattr(link, "occurrence", None)
    if body is None or occ is None or pt.x is None or pt.y is None:
        return None

    tx, ty = pt.x, pt.y

    best_edge = None
    best_dist = 1e9
    best_radius = 1e9  # prefer the *smaller* circle (hole)

    for edge in body.edges:
        geom = edge.geometry
        if not isinstance(geom, adsk.core.Circle3D):
            continue

        center = geom.center
        cx, cy = center.x, center.y
        r = geom.radius

        dx = cx - tx
        dy = cy - ty
        d2 = dx*dx + dy*dy

        if d2 <= tol*tol:
            if d2 < best_dist or (abs(d2 - best_dist) < 1e-9 and r < best_radius):
                best_dist = d2
                best_radius = r
                best_edge = edge

    if best_edge is None:
        return None

    # Return edge *in assembly context* (proxy)
    return best_edge.createForAssemblyContext(occ)

def circle_circle_intersection(c1x, c1y, r1, c2x, c2y, r2, solution=1):
    """
    Simple circle–circle intersection in 2D.
    Returns one intersection point (x, y) or None if no real intersection.
    """
    dx = c2x - c1x
    dy = c2y - c1y
    d = math.hypot(dx, dy)

    # No solution / degenerate
    if d < 1e-12:
        return None
    if d > (r1 + r2) + 1e-12:
        return None
    if d < abs(r1 - r2) - 1e-12:
        return None

    # Distance from c1 along the line to midpoint of intersections
    a = (r1*r1 - r2*r2 + d*d) / (2.0 * d)
    h_sq = r1*r1 - a*a
    if h_sq < 0:
        h_sq = 0.0
    h = math.sqrt(h_sq)

    # Point along the line between centers
    xm = c1x + a * dx / d
    ym = c1y + a * dy / d

    # Perpendicular direction
    rx = -dy / d
    ry =  dx / d

    # Choose one of the two intersections
    xi = xm + solution * h * rx
    yi = ym + solution * h * ry

    return xi, yi

def solve_link_pose_from_two_points(link, p1, p2):
    """
    Given a link and two points on that link:
      - p1, p2: Point2D instances on `link` (with local (u,v) and global (x,y))
    solve the 2D rigid transform (R,t) and update ALL points on the link.

    Returns True on success, False on failure (degenerate geometry).
    """
    # Local coordinates
    u1, v1 = p1.u, p1.v
    u2, v2 = p2.u, p2.v

    # Global coordinates
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y

    # Local segment 1→2
    vLx = u2 - u1
    vLy = v2 - v1
    if abs(vLx) < 1e-12 and abs(vLy) < 1e-12:
        return False

    theta_L = math.atan2(vLy, vLx)

    # Global segment 1→2
    vGx = x2 - x1
    vGy = y2 - y1
    if abs(vGx) < 1e-12 and abs(vGy) < 1e-12:
        return False

    theta_G = math.atan2(vGy, vGx)

    # Rotation from local → global
    theta = theta_G - theta_L
    c = math.cos(theta)
    s = math.sin(theta)

    # Translation: [x1; y1] = [tx; ty] + R * [u1; v1]
    tx = x1 - (c * u1 - s * v1)
    ty = y1 - (s * u1 + c * v1)

    # Apply to all points of this link
    for p in link.pts.values():
        x = tx + c * p.u - s * p.v
        y = ty + s * p.u + c * p.v
        p.set_global(x, y)

    return True

# Initialize the global variables for the Application and UserInterface objects.
app = adsk.core.Application.get()
ui  = app.userInterface

def run(context):
    global design, root_comp
    try:
        # We don't need the design yet; just ensure Fusion is running.
        if not app.activeProduct:
            ui.messageBox("No active product. Open a design or assembly first.")
            return
        
        # Get active design
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design open.")
            return

        root_comp = design.rootComponent

        # -------------------------------
        # Load external JSON file
        # -------------------------------
        script_dir = os.path.dirname(__file__)
        #json_path = os.path.join(script_dir, "4BARMECH.json")
        json_path = os.path.join(script_dir, "6BARMECH_WATT_I.json")

        with open(json_path, "r") as f:
            raw = json.load(f)
        mech = Mechanism.from_json(raw)
       
        mech.postion(theta_crank=math.pi/6)
        mech.generate()
        mech.connect()

        ui.messageBox(
            "Mechanism parsed successfully!\n\n"
            "Links:   {}\n"
            "Joints:  {}\n"
            "Dyads:   {}\n"
            "Crank:   {}\n"
            "Crank A: {}\n".format(
                len(mech.links),
                len(mech.joints),
                len(mech.dyads),
                "Yes" if mech.crank is not None else "No",
                str(mech.crank.link.pts["A"].x) + ", " + str(mech.crank.link.pts["A"].y) if mech.crank is not None else "N/A"
            )
        )

    except:  
        if ui:
            ui.messageBox("Error:\n{}".format(traceback.format_exc()))