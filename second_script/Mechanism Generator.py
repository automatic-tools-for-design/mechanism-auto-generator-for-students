"""This file acts as the main module for this script."""

import traceback
from sys import intern

import adsk.core
import adsk.fusion
import os
import json
import math
import platform
from collections import Counter

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

    def generate(self,joints,geometry):
        global root_comp  # use global design component

        # 1) Collect points with global coordinates
        global_pts = [p for p in self.pts.values() if p.x is not None and p.y is not None]
        if len(global_pts) == 0:
            return

        pts_xy = [(p.x, p.y) for p in global_pts]
        names=self.pts.keys()
        hole_type=[]
        for name in names:
            if name[-1]=='*':
                hole_type.append(1)
            else:
                hole_type.append(0)

        # 2) Radii and thickness (tune as you like)
        link_radius    = geometry.link_radius   # outer pad radius
        hole_radius    = geometry.hole_radius # joint hole
        link_thickness = geometry.link_thickness   # extrusion thickness AND plane spacing

        # 3) Create a new component for this link
        occs = root_comp.occurrences
        transform = adsk.core.Matrix3D.create()
        occ = occs.addNewComponent(transform)
        comp = occ.component
        comp.name=f'Link {self.id}'
        self.component = comp   # store for later (joints)
        self.occurrence = occ        # ⬅️ important: used by connect()
        self.body = None        # will fill in after extrude

        # 4) Choose sketch plane based on link.plane index, in THIS component
        base_plane  = comp.xYConstructionPlane
        plane_index = getattr(self, "plane", 0)

        #ui.messageBox(f'{joints}')

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
        outer_circles = create_joint_circles(comp, hole_type,sketch, pts_xy, link_radius, hole_radius)
        # if plane_index%1==0: #on even planes create circular holes
        #     outer_circles = create_joint_circles(comp,sketch, pts_xy, link_radius, hole_radius)
        # else:
        #     ui.messageBox("error: plane index must be an integer")

        #TODO potential bug here- what if a joint is created between two links that are not in ajacent planes (say planes 0 and 2?) then both holes will be circular

        #TODO potential bug here- what if 3 (or more) joints are colocated at the same point? this could cause issues.

        # 7) Outer outline (if only one joint, just pad+hole, no outline)
        if len(pts_xy) > 1:
            build_link_outline(sketch, pts_xy, link_radius, outer_circles)

        # 8) Extrude outer loop minus inner circles INSIDE THIS COMPONENT
        body = extrude_largest_profile(comp, sketch, link_thickness)
        outer_circles = create_joint_squares(sketch, pts_xy, link_radius, hole_radius) #draws square that can be extruded later if needed
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

    def makepin(self,idx,num_links,name,geometry):
        #make a pin
        global root_comp  # use global design component

        occs = root_comp.occurrences
        transform = adsk.core.Matrix3D.create()
        occ = occs.addNewComponent(transform)
        comp = occ.component
        comp.name=f'Joint {name}'

        base_plane  = comp.xYConstructionPlane
        planes = comp.constructionPlanes
        offset_val = adsk.core.ValueInput.createByReal(-2 * geometry.link_thickness)
        p_input = planes.createInput()
        p_input.setByOffset(base_plane, offset_val)
        sketch_plane = planes.add(p_input)


        sketch = comp.sketches.add(sketch_plane)

        square_peg_edge_length=5

        #key dims
        width=square_peg_edge_length
        snap_thickness=1.5
        tab_length=1
        tab_height=2
        radius=1
        base_height=1+geometry.link_thickness*num_links
        length_to_tab=7.625+geometry.link_thickness*num_links
        #location to put pin
        starting_x=-25-10*idx
        starting_y=0
        starting_z=0

        #pick out the key points for generation
        origin=adsk.core.Point3D.create(starting_x,starting_y,starting_z)
        p1=adsk.core.Point3D.create(starting_x+width/2,starting_y,starting_z)
        p1m=adsk.core.Point3D.create(starting_x-width/2,starting_y,starting_z)
        p2=adsk.core.Point3D.create(starting_x+width/2,starting_y+length_to_tab,starting_z)
        p2m = adsk.core.Point3D.create(starting_x - width / 2, starting_y + length_to_tab, starting_z)
        p3=adsk.core.Point3D.create(starting_x+width/2+tab_length,starting_y+length_to_tab,starting_z)
        p3m = adsk.core.Point3D.create(starting_x - width / 2 - tab_length, starting_y + length_to_tab, starting_z)
        p4=adsk.core.Point3D.create(starting_x+width/2,starting_y+length_to_tab+tab_height,starting_z)
        p4m = adsk.core.Point3D.create(starting_x - width / 2, starting_y + length_to_tab + tab_height,
                                      starting_z)

        p5=adsk.core.Point3D.create(starting_x+width/2-snap_thickness,starting_y+length_to_tab+tab_height,starting_z)
        p5m = adsk.core.Point3D.create(starting_x - width / 2 + snap_thickness, starting_y + length_to_tab + tab_height,
                                      starting_z)
        radius_center=adsk.core.Point3D.create(starting_x,starting_y+base_height+radius,starting_z)
        p6=adsk.core.Point3D.create(starting_x+radius,starting_y+base_height+radius,starting_z)
        p6m = adsk.core.Point3D.create(starting_x - radius, starting_y + base_height + radius, starting_z)
        p7=adsk.core.Point3D.create(starting_x,starting_y+base_height,starting_z)

        #draw straight lines
        lines=sketch.sketchCurves.sketchLines

        lines.addByTwoPoints(p1,p2)
        lines.addByTwoPoints(p2, p3)
        lines.addByTwoPoints(p3, p4)
        lines.addByTwoPoints(p4, p5)
        lines.addByTwoPoints(p5, p6)

        #draw the curve
        arcs=sketch.sketchCurves.sketchArcs
        arcs.addByCenterStartEnd(radius_center,p6m,p6)

        lines.addByTwoPoints(p5m, p6m)
        lines.addByTwoPoints(p4m, p5m)
        lines.addByTwoPoints(p3m, p4m)
        lines.addByTwoPoints(p2m, p3m)
        lines.addByTwoPoints(p1m, p2m)

        lines.addByTwoPoints(p1m,p1)

        extrudes = comp.features.extrudeFeatures
        distance = adsk.core.ValueInput.createByReal(square_peg_edge_length)


        ext_input = extrudes.createInput(
            sketch.profiles.item(0),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        )

        ext_input.setDistanceExtent(False, distance)

        ext = extrudes.add(ext_input)

class Group:
    """
    Base class for mechanism groups (dyad, classIII, classIV, ...).
    Provides a factory that returns the right derived class based on JSON "type".
    """
    TYPE_MAP = {}  # filled after class definitions

    def __init__(self, group_id, links, internal=None, external=None, solution=1):
        self.id = group_id
        self.links = links
        # internal/external are LISTS for every group type
        self.internal = internal if internal is not None else []
        self.external = external if external is not None else []
        self.solution = solution   # can be int (legacy) OR dict {"circle":..,"phi":..}


    @staticmethod
    def from_json(group_id, data, links, joints):
        gtype = data.get("type", "dyad")  # default to dyad if omitted
        cls = Group.TYPE_MAP.get(gtype)
        if cls is None:
            raise ValueError(f'Unknown group type "{gtype}" in GROUPS["{group_id}"]')
        return cls._from_json(group_id, data, links, joints)

    @staticmethod
    def _from_json(group_id, data, links, joints):
        raise NotImplementedError
    
class Dyad(Group):
    @staticmethod
    def _from_json(dyad_id, data, links, joints):
        link_objs = [links[lid] for lid in data.get("links", [])]

        internal = []
        if "internal" in data:
            internal = [joints[data["internal"]]]   # <- list of 1

        external = [joints[jid] for jid in data.get("external", [])]

        solution = data.get("solution", 1)
        return Dyad(dyad_id, link_objs, internal=internal, external=external, solution=solution)
    
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
            joint = self.internal[0]

            # internal joint must use same point label on both links

            name_i = joint.pt_i_name
            name_j = joint.pt_j_name


            #todo: fix this mess
            if name_i != name_j:
                if len(name_i) > len(name_j):
                    name_i, name_j = name_j, name_i
                if name_j == name_i+'*':
                    pass
                else:
                    raise RuntimeError(
                    f"Joint {joint.id}: mismatched point labels {name_i} vs {name_j}"
                    )
            if name_i[-1]=='*' and name_j[-1]=='*':
                ui.messageBox(
                    f"WARNING: Joint {joint.id}- Both joints end in '*' and will not be able to rotate. Please revise"
                )

            pivot_name = name_i

            # 1) Ground link points: local == global
            for p in ground_link.pts.values():
                p.set_global(p.u, p.v)

            # Global pivot on ground
            try:
                p_g = ground_link.pts[pivot_name]
            except:
                p_g = ground_link.pts[pivot_name+'*']
            xg, yg = p_g.x, p_g.y

            # Local pivot on crank
            p_c = crank_link.pts[pivot_name]
            uc, vc = p_c.u, p_c.v

            #check if points are actually the same for pivot
            if xg != uc or yg != vc:
                raise RuntimeError(
                    f"cordinates for first dyad do not match"
                )

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

            # internal is a list (by design). Dyad has exactly one internal joint.
            if len(self.internal) != 1:
                raise RuntimeError(f"Dyad {self.id}: expected exactly 1 internal joint, got {len(self.internal)}")

            Jint = self.internal[0]   # <-- FIX

            # Internal joint points on the two links
            int_L1 = Jint.pt_i
            int_L2 = Jint.pt_j
            L1     = Jint.link_i
            L2     = Jint.link_j

            #so right here it seems like we are using relative cordinates, not absolute ones
            #int_L1.u, int_L1.v are the values we read off of the JSON file for the first time the joint appears
            #int_L1.u, int_L1.v are the values we read off of the JSON file for the second time the joint appears
            #ui.messageBox(f'{int_L1.u, int_L1.v, int_L2.u, int_L2.v}')


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


                #these seem to be the first time the joint is found?
                #ui.messageBox(f'{pi.x, pi.y, pj.x, pj.y}')
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

    
    # Register concrete group types (keep it simple, no decorators)

class ClassIV(Group):
    @staticmethod
    def _from_json(gid, data, links, joints):
        link_objs = [links[lid] for lid in data.get("links", [])]

        internal = [joints[jid] for jid in data.get("internal", [])]  # list of Joint
        external = [joints[jid] for jid in data.get("external", [])]  # list of Joint

        solution = data.get("solution", 1)
        return ClassIV(gid, link_objs, internal=internal, external=external, solution=solution)

    def solve_position(self, theta_crank=None):
        # -----------------------------
        # 0) Basic validation
        # -----------------------------
        group_links = list(self.links)
        group_links_set = set(group_links)

        internal_joints = list(self.internal)
        external_joints = list(self.external)

        if len(group_links) != 4:
            raise RuntimeError(f"ClassIV {self.id}: expected 4 links, got {len(group_links)}")
        if len(internal_joints) != 4:
            raise RuntimeError(f"ClassIV {self.id}: expected 4 internal joints, got {len(internal_joints)}")
        if len(external_joints) != 2:
            raise RuntimeError(f"ClassIV {self.id}: expected 2 external joints, got {len(external_joints)}")

        # -----------------------------
        # 1) Propagate known external joint global coords
        #    (at least one side of each external joint should already be known)
        # -----------------------------
        for J in external_joints:
            pi = J.pt_i
            pj = J.pt_j
            if pi.x is not None and pj.x is None:
                pj.set_global(pi.x, pi.y)
            elif pj.x is not None and pi.x is None:
                pi.set_global(pj.x, pj.y)

        # After propagation, the point on the GROUP link side must be known:
        ext_inside = []
        for J in external_joints:
            lk_in = group_side_link_of_external_joint(J, group_links_set)
            if lk_in is None:
                raise RuntimeError(f"ClassIV {self.id}: external joint {J.id} does not touch any group link.")
            p_in = point_on_link(J, lk_in)
            if p_in is None or p_in.x is None:
                raise RuntimeError(f"ClassIV {self.id}: external joint {J.id} group-side point has no global coords.")
            ext_inside.append((J, lk_in, p_in))

        # -----------------------------
        # 2) Compute degrees within the group (internal + external)
        #    Identify ternaries (deg=3) and binaries (deg=2)
        # -----------------------------
        deg = {lk.id: 0 for lk in group_links}
        for J in internal_joints + external_joints:
            if J.link_i.id in deg: deg[J.link_i.id] += 1
            if J.link_j.id in deg: deg[J.link_j.id] += 1

        vals = sorted(deg.values())
        if vals != [2, 2, 3, 3]:
            raise RuntimeError(
                f"ClassIV {self.id}: expected degrees [2,2,3,3], got {vals}. "
                + "Degrees: " + ", ".join([f"{k}:{v}" for k, v in deg.items()])
            )

        ternaries = [lk for lk in group_links if deg[lk.id] == 3]
        binaries  = [lk for lk in group_links if deg[lk.id] == 2]

        # -----------------------------
        # 3) Determine which ternary corresponds to which external pivot
        #    (each external joint attaches to exactly one group link; should be ternary)
        # -----------------------------
        # Map: ternary -> (external_joint, external_point_on_ternary)
        ternary_ext = {}
        for (J, lk_in, p_in) in ext_inside:
            if lk_in not in ternaries:
                # In a valid Class IV Watt group, externals hit ternaries.
                # If not, it's a different topology than this solver assumes.
                raise RuntimeError(
                    f"ClassIV {self.id}: external joint {J.id} attaches to non-ternary link {lk_in.id}."
                )
            ternary_ext[lk_in] = (J, p_in)

        if len(ternary_ext) != 2:
            raise RuntimeError(f"ClassIV {self.id}: expected external joints to hit the two ternaries (got {len(ternary_ext)}).")

        T1, T2 = list(ternary_ext.keys())
        Jext1, P1 = ternary_ext[T1][0], ternary_ext[T1][1]  # P1 is Point2D on T1 with global known
        Jext2, P2 = ternary_ext[T2][0], ternary_ext[T2][1]

        P1_xy = (P1.x, P1.y)
        P2_xy = (P2.x, P2.y)

        # -----------------------------
        # 4) Build incidence: for each binary link, find the two internal joints that connect it to the ternaries
        # -----------------------------
        # For each binary link, collect internal joints touching it
        bin_to_internal = {b: [] for b in binaries}
        for J in internal_joints:
            if J.link_i in bin_to_internal:
                bin_to_internal[J.link_i].append(J)
            if J.link_j in bin_to_internal:
                bin_to_internal[J.link_j].append(J)

        # Each binary should have exactly 2 internal joints (to the two ternaries)
        for b in binaries:
            if len(bin_to_internal[b]) != 2:
                raise RuntimeError(
                    f"ClassIV {self.id}: binary link {b.id} does not have exactly 2 internal joints inside group."
                )

        # Choose cut vs remaining binary deterministically (by id)
        binaries_sorted = sorted(binaries, key=lambda lk: lk.id)
        B_cut = binaries_sorted[0]
        B_rem = binaries_sorted[1]

        # For each ternary, find its joint to B_cut and to B_rem
        def internal_joint_between(link_a, link_b):
            for J in internal_joints:
                if (J.link_i is link_a and J.link_j is link_b) or (J.link_j is link_a and J.link_i is link_b):
                    return J
            return None

        J_T1_cut = internal_joint_between(T1, B_cut)
        J_T2_cut = internal_joint_between(T2, B_cut)
        J_T1_rem = internal_joint_between(T1, B_rem)
        J_T2_rem = internal_joint_between(T2, B_rem)

        if None in [J_T1_cut, J_T2_cut, J_T1_rem, J_T2_rem]:
            raise RuntimeError(f"ClassIV {self.id}: could not identify required internal joints for cut/rem binaries.")

        # Points on ternaries for those joints
        T1_cut_pt = point_on_link(J_T1_cut, T1)
        T2_cut_pt = point_on_link(J_T2_cut, T2)
        T1_rem_pt = point_on_link(J_T1_rem, T1)
        T2_rem_pt = point_on_link(J_T2_rem, T2)

        # Points on binaries for those joints (useful for lengths)
        Bcut_pt1 = point_on_link(J_T1_cut, B_cut)
        Bcut_pt2 = point_on_link(J_T2_cut, B_cut)
        Brem_pt1 = point_on_link(J_T1_rem, B_rem)
        Brem_pt2 = point_on_link(J_T2_rem, B_rem)

        # -----------------------------
        # 5) Known lengths from LOCAL geometry (no labels)
        # -----------------------------
        # - ground length between external pivots is known in global (but not needed as "length")
        # - r1: distance on T1 from external pivot point to its rem joint (LOCAL)
        # - r2: distance on T2 from external pivot point to its rem joint (LOCAL)
        # - L_rem: length of remaining binary (LOCAL)
        # - L_cut: length of cut binary (LOCAL)
        P1_local = P1  # Point2D on T1 (has u,v and x,y)
        P2_local = P2  # Point2D on T2

        r1 = math.hypot(T1_rem_pt.u - P1_local.u, T1_rem_pt.v - P1_local.v)
        r2 = math.hypot(T2_rem_pt.u - P2_local.u, T2_rem_pt.v - P2_local.v)

        L_rem = math.hypot(Brem_pt2.u - Brem_pt1.u, Brem_pt2.v - Brem_pt1.v)
        L_cut = math.hypot(Bcut_pt2.u - Bcut_pt1.u, Bcut_pt2.v - Bcut_pt1.v)

        # Local direction vector on T1 from external pivot to rem joint
        v1x = T1_rem_pt.u - P1_local.u
        v1y = T1_rem_pt.v - P1_local.v

        # -----------------------------
        # 6) Define closure error f(phi):
        #    - rotate T1 about P1 by phi -> gives Q1 (T1_rem global)
        #    - solve Q2 (T2_rem global) from circle-circle intersection:
        #          |Q2 - P2| = r2
        #          |Q2 - Q1| = L_rem
        #    - then both ternary poses are known (from two points each)
        #    - compute cut joint points global and check:
        #          |T1_cut - T2_cut| == L_cut
        # -----------------------------
        # solution can be int (legacy) OR dict {"circle": ±1, "phi": 0/1}
        sol = self.solution

        if isinstance(sol, dict):
            sol_sign = +1 if int(sol.get("circle", 1)) >= 0 else -1
            phi_idx  = int(sol.get("phi", 0))
        else:
            # legacy behavior (your old JSON where solution was +/- 1)
            sol_sign = +1 if int(sol) >= 0 else -1
            phi_idx  = 0

        def eval_f(phi, commit=False):
            # Snapshot BEFORE mutation (for pure evaluation during scanning)
            snaps = snapshot_many([T1, T2, B_cut, B_rem])

            try:
                # Q1 from rotating T1's local vector v1 around P1 global
                rvx, rvy = rot2(phi, v1x, v1y)
                Q1x = P1_xy[0] + rvx
                Q1y = P1_xy[1] + rvy

                # Solve Q2 from intersection of circles:
                sol_cc = circle_circle_intersection(
                    Q1x, Q1y, L_rem,
                    P2_xy[0], P2_xy[1], r2,
                    solution=sol_sign
                )
                if sol_cc is None:
                    return None, None
                Q2x, Q2y = sol_cc

                # Solve pose of T1 from its external pivot and rem joint
                T1_ext_pt = P1_local
                T1_ext_pt.set_global(P1_xy[0], P1_xy[1])
                T1_rem_pt.set_global(Q1x, Q1y)
                ok1 = solve_link_pose_from_two_points(T1, T1_ext_pt, T1_rem_pt)
                if not ok1:
                    return None, None

                # Solve pose of T2 from its external pivot and rem joint
                T2_ext_pt = P2_local
                T2_ext_pt.set_global(P2_xy[0], P2_xy[1])
                T2_rem_pt.set_global(Q2x, Q2y)
                ok2 = solve_link_pose_from_two_points(T2, T2_ext_pt, T2_rem_pt)
                if not ok2:
                    return None, None

                # Closure error on cut binary length
                d = math.hypot(T1_cut_pt.x - T2_cut_pt.x, T1_cut_pt.y - T2_cut_pt.y)
                fval = d - L_cut

                return fval, (Q1x, Q1y, Q2x, Q2y)

            finally:
                # Restore unless this evaluation is meant to "commit" the final pose
                if not commit:
                    restore_many(snaps)
        # -----------------------------
        # 7) Find ALL phi roots where f(phi)=0, then select by phi_idx
        # -----------------------------
        samples = 1440  # denser sampling to avoid missing roots
        phis = [2.0 * math.pi * i / samples for i in range(samples + 1)]

        vals = []
        for phi in phis:
            f, _ = eval_f(phi)
            vals.append(f)

        # Collect all brackets (sign-change + near-zero + touch roots)
        brackets = []
        eps = 1e-6

        for i in range(samples):
            f0 = vals[i]
            f1 = vals[i + 1]
            if f0 is None or f1 is None:
                continue

            # endpoint near-zero checks
            if abs(f0) < eps:
                brackets.append((phis[i], phis[i]))
                continue
            if abs(f1) < eps:
                brackets.append((phis[i+1], phis[i+1]))
                continue

            # sign-change bracket
            if f0 * f1 < 0:
                brackets.append((phis[i], phis[i + 1]))
                continue

            # TOUCH root detection: check midpoint
            m = 0.5 * (phis[i] + phis[i + 1])
            fm, _ = eval_f(m)
            if fm is not None and abs(fm) < eps:
                brackets.append((phis[i], phis[i + 1]))


        def bisect_root(a, b, iters=50):
            if a == b:
                return a
            fa, _ = eval_f(a)
            fb, _ = eval_f(b)
            if fa is None or fb is None:
                return None
            for _ in range(iters):
                m = 0.5 * (a + b)
                fm, _ = eval_f(m)
                if fm is None:
                    return None
                if abs(fm) < 1e-8:
                    return m
                if fa * fm < 0:
                    b = m
                    fb = fm
                else:
                    a = m
                    fa = fm
            return 0.5 * (a + b)

        # Solve each bracket and deduplicate roots
        roots = []
        for a, b in brackets:
            r = bisect_root(a, b)
            if r is None:
                continue
            r = r % (2.0 * math.pi)
            # dedup with wrap-around tolerance
            if all(abs(((r - rr + math.pi) % (2*math.pi)) - math.pi) > 1e-3 for rr in roots):
                roots.append(r)

        roots.sort()

        ui.messageBox(
            f"ClassIV {self.id}\n"
            f"circle={sol_sign}\n"
            f"roots found = {len(roots)}\n"
            f"roots (deg) = {[round(r*180/math.pi, 3) for r in roots]}"
        )

        if len(roots) == 0:
            raise RuntimeError(f"ClassIV {self.id}: no closure roots found for circle={sol_sign}")

        # Only allow phi = 0 or 1
        if phi_idx not in (0, 1):
            raise RuntimeError(f"ClassIV {self.id}: phi must be 0 or 1 (got {phi_idx})")

        if phi_idx >= len(roots):
            raise RuntimeError(
                f"ClassIV {self.id}: requested phi={phi_idx} but only {len(roots)} root(s) exist for circle={sol_sign}. "
                f"Roots (rad): {['{:.6f}'.format(r) for r in roots]}"
            )

        phi_star = roots[phi_idx]

        #ui.messageBox(f"ClassIV {self.id}: circle={sol_sign}, phi_idx={phi_idx}, roots={len(roots)}")


        # -----------------------------
        # 8) Final evaluation at phi_star to SET all point globals
        # -----------------------------
        f_final, aux = eval_f(phi_star, commit=True)
        if f_final is None:
            raise RuntimeError(f"ClassIV {self.id}: final phi gave invalid configuration.")
        # At this point T1 and T2 poses have been solved and all their points updated.

        # Optional: also solve remaining binary pose (nice to have, not required for next groups)
        # Use the joint points on the binary and the already-updated joint points on ternaries.
        # Set binary endpoints globals from the joint-matched points on ternaries:
        Brem_pt1.set_global(T1_rem_pt.x, T1_rem_pt.y)
        Brem_pt2.set_global(T2_rem_pt.x, T2_rem_pt.y)
        solve_link_pose_from_two_points(B_rem, Brem_pt1, Brem_pt2)

        # Optional: solve cut binary pose too (after closure)
        Bcut_pt1.set_global(T1_cut_pt.x, T1_cut_pt.y)
        Bcut_pt2.set_global(T2_cut_pt.x, T2_cut_pt.y)
        solve_link_pose_from_two_points(B_cut, Bcut_pt1, Bcut_pt2)

        return

Group.TYPE_MAP = {
    "dyad": Dyad,
    "classIV": ClassIV,
}

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

class Geometry:
    def __init__(self, link_radius    = 0.5 ,hole_radius=3.75, link_thickness = 0.5 ):

        self.link_radius = float(link_radius)
        self.hole_radius = float(hole_radius)
        self.link_thickness = float(link_thickness)


    @staticmethod
    def from_json(raw_geometry):
        link_radius = raw_geometry.get("link_radius", 0.5)
        hole_radius = 3.75
        link_thickness = raw_geometry.get("link_thickness", 0.5)
        if link_radius<5:
            ui.messageBox('WARNING! Link Radius must be greater than 5')
        return Geometry(link_radius,hole_radius,link_thickness)

class Mechanism:
    def __init__(self, links, joints, groups, geometry, crank=None):
        self.links = links        # dict[str : Link]
        self.joints = joints      # dict[str : Joint]
        self.groups = groups        # list[Dyad]   <--- ordered list
        self.crank = crank        # Crank or None
        self.geometry= geometry

    @classmethod
    def from_json(cls, raw):
        raw_links = raw.get("LINKS", {})
        raw_joints = raw.get("JOINTS", {})
        raw_groups = raw.get("GROUPS", {})
        raw_crank = raw.get("CRANK", None)
        raw_geometry=raw.get("GEOMETRY",{})

        geometry=Geometry.from_json(raw_geometry)

        # 1. Links
        links = {
            lid: Link.from_json(lid, ldata) for lid, ldata in raw_links.items()
        }

        # 2. Joints
        joints = {
            jid: Joint.from_json(jid, jdata, links) for jid, jdata in raw_joints.items()
        }

        # 3. Groups (ordered list!)
        groups = []
        for did, ddata in raw_groups.items():   # preserves JSON order
            group = Group.from_json(did, ddata, links, joints)
            groups.append(group)

        # 4. Crank
        crank = Crank.from_json(raw_crank, links, joints)

        # return assembled mechanism
        return cls(links, joints, groups, geometry, crank)

    def postion(self, theta_crank):
        """
        Solve the position problem dyad-by-dyad.

        For each dyad:
          - If it is the driving dyad (one ground link, one crank link),
            solve it as a special case: pure rotation of the crank link.
          - Otherwise, invoke the ordinary dyad position solver
            (to be implemented) which uses external joints.
        """
        for i, groups in enumerate(self.groups):
            if i == 0:
                # Driving dyad: use given crank angle
                groups.solve_position(theta_crank=theta_crank)
            else:
                # Ordinary dyads: cascade P-problem (stub for now)
                groups.solve_position()

    def generate(self):
        """
        Generate the mechanism in Fusion 360.

        This is a stub for now.
        """
        for link in self.links.values():   
            link.generate(self.joints, self.geometry)

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
        joint_list=[]
        for joint in self.joints.values():
            joint_list.append(joint.pt_j_name)

        counts = Counter(joint_list)
        num_pins=0
        for item in counts:

            joint.makepin(num_pins,counts[item],item,self.geometry)
            num_pins+=1

        for joint in self.joints.values():
            #print(joint)
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

def create_joint_circles(comp,hole_type,sketch, pts_xy, link_radius, hole_radius):
    """
    Draw outer + inner circles at joint centers.
    Returns dict[(x,y)] -> outer SketchCircle.
    """
    x=pts_xy[0]
    y=pts_xy[1]
    circles = sketch.sketchCurves.sketchCircles
    rectangle=sketch.sketchCurves.sketchLines
    outer_circles = {}
    idx=0
    for (x, y) in pts_xy:
        if hole_type[idx]==0:
            center = adsk.core.Point3D.create(x, y, 0)
            outer = circles.addByCenterRadius(center, link_radius)
            outer_circles[(x, y)] = outer
            circles.addByCenterRadius(center, hole_radius)
        if hole_type[idx]==1:
            distance = 5.125  # if we change the hole radius, this should change as well
            center = adsk.core.Point3D.create(x, y, 0)
            corner1 = adsk.core.Point3D.create(x + distance / 2, y + distance / 2, 0)
            corner2 = adsk.core.Point3D.create(x - distance / 2, y - distance / 2, 0)
            outer = circles.addByCenterRadius(center, link_radius)
            outer_circles[(x, y)] = outer
            rectangle.addTwoPointRectangle(corner1, corner2)
        idx+=1
    return outer_circles

def create_joint_squares(sketch,pts_xy, link_radius, hole_radius):
    """
    Draw outer circle and inner rectangle
    Returns dict[(x,y)] -> outer SketchCircle.
    """
    circles = sketch.sketchCurves.sketchCircles
    rectangle=sketch.sketchCurves.sketchLines
    # axes=comp.constructionAxes
    # axisInput = axes.createInput()
    outer_circles = {}
    for (x, y) in pts_xy:
        distance=5.125 #if we change the hole radius, this should change as well
        center = adsk.core.Point3D.create(x, y, 0)

        # axisInput.setByTwoPoints(center, center2)
        # axes.add(axisInput)

        corner1= adsk.core.Point3D.create(x+distance/2, y+distance/2, 0)
        corner2= adsk.core.Point3D.create(x-distance/2, y-distance/2, 0)
        outer = circles.addByCenterRadius(center, link_radius)


        # axisInput.setByEdge(outer)
        # axes.add(axisInput)

        outer_circles[(x, y)] = outer
        rectangle.addTwoPointRectangle(corner1, corner2)
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

def point_on_link(J, link):
    """Return the Point2D object on `link` that participates in joint J."""
    if J.link_i is link:
        return J.pt_i
    if J.link_j is link:
        return J.pt_j
    return None

def group_side_link_of_external_joint(J, group_links_set):
    """
    For an external joint, return the link that is inside the group.
    We do NOT care what the other link is (ground or moving).
    """
    if J.link_i in group_links_set and J.link_j not in group_links_set:
        return J.link_i
    if J.link_j in group_links_set and J.link_i not in group_links_set:
        return J.link_j
    # If malformed (both in group or both out), still try to handle:
    if J.link_i in group_links_set:
        return J.link_i
    if J.link_j in group_links_set:
        return J.link_j
    return None

def rot2(phi, vx, vy):
    c = math.cos(phi)
    s = math.sin(phi)
    return (c*vx - s*vy, s*vx + c*vy)

def dist2(p, q):
    return math.hypot(p[0]-q[0], p[1]-q[1])

def debug_classiv_topology(group, topo):
    msg = []
    msg.append(f"ClassIV {group.id} topology:")
    msg.append("Links: " + ", ".join([lk.id for lk in topo["links"]]))
    msg.append("Internal joints: " + ", ".join([J.id for J in topo["internal_joints"]]))
    msg.append("External joints: " + ", ".join([J.id for J in topo["external_joints"]]))
    msg.append("Degrees (internal only): " + ", ".join([f"{lk.id}:{topo['degree_by_link'][lk.id]}" for lk in topo["links"]]))
    msg.append("Ternaries: " + ", ".join([lk.id for lk in topo["ternaries"]]))
    msg.append("Binaries: " + ", ".join([lk.id for lk in topo["binaries"]]))
    msg.append(f"Disconnected binary: {topo['disconnected_binary'].id}")
    msg.append(f"Disconnected joint: {topo['disconnected_joint'].id}")
    ui.messageBox("\n".join(msg))

def snapshot_link_globals(link):
    """Return dict {pt_name: (x,y)} for this link."""
    snap = {}
    for name, p in link.pts.items():
        snap[name] = (p.x, p.y)
    return snap

def restore_link_globals(link, snap):
    """Restore globals from snapshot dict."""
    for name, (x, y) in snap.items():
        p = link.pts[name]
        # Restore exactly (including None)
        p.x = x
        p.y = y

def snapshot_many(links):
    return {lk: snapshot_link_globals(lk) for lk in links}

def restore_many(snaps):
    for lk, snap in snaps.items():
        restore_link_globals(lk, snap)

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

        if platform.system() == 'Windows':
            downloads_path = os.path.join(os.getenv('USERPROFILE'), 'Downloads')
        else:
            downloads_path = os.path.join(os.path.expanduser('~'), 'Downloads')
        # -------------------------------
        # Load external JSON file
        # Load external JSON file
        # -------------------------------
        script_dir = os.path.dirname(__file__)
        json_path = os.path.join(script_dir, "4BARMECH.json")
        #json_path = os.path.join(script_dir, "6BARMECH_WATT_I.json")
        #json_path = os.path.join(script_dir, "6BARMECH_STEPHENSON_II.json")
        #json_path = os.path.join(script_dir, "Theo_Jansen.json")

        with open(json_path, "r") as f:
            raw = json.load(f)
        mech = Mechanism.from_json(raw)
       
        mech.postion(theta_crank=0)
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
                len(mech.groups),
                "Yes" if mech.crank is not None else "No",
                str(mech.crank.link.pts["A"].x) + ", " + str(mech.crank.link.pts["A"].y) if mech.crank is not None else "N/A"
            )
        )


        result = ui.messageBox(
            "Do you want to Export STL files of your all parts to the Downloads folder?",
            "Export STL files",
            adsk.core.MessageBoxButtonTypes.YesNoButtonType
        )

        if result==2:
            # create a single exportManager instance
            exportMgr = design.exportManager
            allOccu = root_comp.allOccurrences
            for occ in allOccu:
                fileName = downloads_path + "/" + occ.component.name

                # create stl exportOptions
                stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                stlExportOptions.sendToPrintUtility = False

                exportMgr.execute(stlExportOptions)
            ui.messageBox(f'Files exported - script complete')
        elif result==3:
            ui.messageBox(f'Script complete')
    except:  
        if ui:
            ui.messageBox("Error:\n{}".format(traceback.format_exc()))

