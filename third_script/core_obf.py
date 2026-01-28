k='solution'
j='external'
i='dyad'
h='plane'
g=property
U='internal'
A2=sorted
A1=isinstance
S='occurrence'
p=True
Q='links'
o=range
n=int
P=.0
b=list
R=', '
L=False
J=getattr
G=staticmethod
H=abs
F=float
D='C'
E=RuntimeError
C=len
A=None
import traceback
from sys import intern
import adsk.core,adsk.fusion,os,json,math as B,platform
from collections import Counter as l
class V:
	def __init__(B,u,v,x=A,y=A):B.u=F(u);B.v=F(v);B.x=A if x is A else F(x);B.y=A if y is A else F(y)
	@G
	def from_json(raw_point):A=raw_point;return V(A[0],A[1])
	def set_global(A,x,y):A.x=F(x);A.y=F(y)
class W:
	def __init__(B,link_id,ground=L,plane=0,pts=A):B.id=link_id;B.ground=bool(ground);B.plane=n(plane);B.pts=pts if pts is not A else{};B.component=A;B.occurrence=A;B.body=A
	@G
	def from_json(link_id,data):
		A=data;B={};C=A.get('pts',{})
		for(D,E)in C.items():B[D]=V.from_json(E)
		return W(link_id,ground=A.get('ground',L),plane=A.get(h,0),pts=B)
	def generate(B,joints,geometry):
		H=geometry;global I;M=[B for B in B.pts.values()if B.x is not A and B.y is not A]
		if C(M)==0:return
		K=[(A.x,A.y)for A in M];N=b(B.pts.keys());G={A:D for A in N}
		for E in joints.values():
			if E.link_i is B and E.pt_i_name in G:G[E.pt_i_name]=E.socket_i
			if E.link_j is B and E.pt_j_name in G:G[E.pt_j_name]=E.socket_j
		W=[G[A]for A in N];O=H.link_radius;X=H.hole_radius;P=H.link_thickness;Y=I.occurrences;Z=adsk.core.Matrix3D.create();Q=Y.addNewComponent(Z);F=Q.component;F.name=f"Link {B.id}";B.component=F;B.occurrence=Q;B.body=A;R=F.xYConstructionPlane;S=J(B,h,0)
		if S==0:T=R
		else:U=F.constructionPlanes;a=adsk.core.ValueInput.createByReal(S*P);V=U.createInput();V.setByOffset(R,a);T=U.add(V)
		L=F.sketches.add(T);c=s(L,K,W,O,X)
		if C(K)>1:u(L,K,O,c)
		d=v(F,L,P);B.body=d
class X:
	def __init__(A,joint_id,link_i,pt_i_name,link_j,pt_j_name,socket_i=D,socket_j=D):A.id=joint_id;A.link_i=link_i;A.pt_i_name=pt_i_name;A.link_j=link_j;A.pt_j_name=pt_j_name;A.socket_i=(socket_i or D).upper();A.socket_j=(socket_j or D).upper()
	@g
	def pt_i(self):return self.link_i.pts[self.pt_i_name]
	@g
	def pt_j(self):return self.link_j.pts[self.pt_j_name]
	@G
	def from_json(joint_id,data,links):
		F=links;E=data
		if A1(E,dict):G=E.get('ends',[])
		else:G=E
		A,B=G;H=A[0];I=A[1];J=A[2]if C(A)>=3 else D;K=B[0];L=B[1];M=B[2]if C(B)>=3 else D;N=F[H];O=F[K];return X(joint_id,N,I,O,L,socket_i=J,socket_j=M)
	def makepin(r,idx,num_links,name,geometry):O=num_links;M=geometry;global I;i=I.occurrences;j=adsk.core.Matrix3D.create();k=i.addNewComponent(j);G=k.component;G.name=f"Joint {name}";l=G.xYConstructionPlane;P=G.constructionPlanes;m=adsk.core.ValueInput.createByReal(-2*M.link_thickness);Q=P.createInput();Q.setByOffset(l,m);n=P.add(Q);N=G.sketches.add(n);R=5;E=R;S=1.5;T=1;J=2;H=1;K=1+M.link_thickness*O;F=M.link_thickness*(O+1)+.125;A=-25-10*idx;B=0;C=0;s=adsk.core.Point3D.create(A,B,C);U=adsk.core.Point3D.create(A+E/2,B,C);V=adsk.core.Point3D.create(A-E/2,B,C);W=adsk.core.Point3D.create(A+E/2,B+F,C);X=adsk.core.Point3D.create(A-E/2,B+F,C);Y=adsk.core.Point3D.create(A+E/2+T,B+F,C);Z=adsk.core.Point3D.create(A-E/2-T,B+F,C);a=adsk.core.Point3D.create(A+E/2,B+F+J,C);b=adsk.core.Point3D.create(A-E/2,B+F+J,C);c=adsk.core.Point3D.create(A+E/2-S,B+F+J,C);d=adsk.core.Point3D.create(A-E/2+S,B+F+J,C);o=adsk.core.Point3D.create(A,B+K+H,C);e=adsk.core.Point3D.create(A+H,B+K+H,C);f=adsk.core.Point3D.create(A-H,B+K+H,C);t=adsk.core.Point3D.create(A,B+K,C);D=N.sketchCurves.sketchLines;D.addByTwoPoints(U,W);D.addByTwoPoints(W,Y);D.addByTwoPoints(Y,a);D.addByTwoPoints(a,c);D.addByTwoPoints(c,e);p=N.sketchCurves.sketchArcs;p.addByCenterStartEnd(o,f,e);D.addByTwoPoints(d,f);D.addByTwoPoints(b,d);D.addByTwoPoints(Z,b);D.addByTwoPoints(X,Z);D.addByTwoPoints(V,X);D.addByTwoPoints(V,U);g=G.features.extrudeFeatures;q=adsk.core.ValueInput.createByReal(R);h=g.createInput(N.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation);h.setDistanceExtent(L,q);u=g.add(h)
class O:
	TYPE_MAP={}
	def __init__(B,group_id,links,internal=A,external=A,solution=1):D=external;C=internal;B.id=group_id;B.links=links;B.internal=C if C is not A else[];B.external=D if D is not A else[];B.solution=solution
	@G
	def from_json(group_id,data,links,joints):
		B=group_id;C=data.get('type',i);D=O.TYPE_MAP.get(C)
		if D is A:raise ValueError(f'Unknown group type "{C}" in GROUPS["{B}"]')
		return D._from_json(B,data,links,joints)
	@G
	def _from_json(group_id,data,links,joints):raise NotImplementedError
class Y(O):
	@G
	def _from_json(dyad_id,data,links,joints):
		B=joints;A=data;D=[links[A]for A in A.get(Q,[])];C=[]
		if U in A:C=[B[A[U]]]
		E=[B[A]for A in A.get(j,[])];F=A.get(k,1);return Y(dyad_id,D,internal=C,external=E,solution=F)
	def solve_position(D,theta_crank=A):
		c=theta_crank;S='*';U=[A for A in D.links if A.ground];V=[A for A in D.links if not A.ground];u=C(U)==1 and C(V)==1
		if u:
			if c is A:raise E('Driving dyad requires theta_crank.')
			W=U[0];d=V[0];N=D.internal[0];G=N.pt_i_name;H=N.pt_j_name
			if G!=H:
				if C(G)>C(H):G,H=H,G
				if H==G+S:0
				else:raise E(f"Joint {N.id}: mismatched point labels {G} vs {H}")
			if G[-1]==S and H[-1]==S:a.messageBox(f"WARNING: Joint {N.id}- Both joints end in '*' and will not be able to rotate. Please revise")
			X=G
			for I in W.pts.values():I.set_global(I.u,I.v)
			try:Y=W.pts[X]
			except:Y=W.pts[X+S]
			e,f=Y.x,Y.y;g=d.pts[X];h,i=g.u,g.v
			if e!=h or f!=i:raise E(f"cordinates for first dyad do not match")
			j=F(c);k=B.cos(j);l=B.sin(j)
			for I in d.pts.values():m=I.u-h;n=I.v-i;v=e+k*m-l*n;w=f+l*m+k*n;I.set_global(v,w)
		else:
			if C(D.external)!=2:return
			o=D.external[0];p=D.external[1]
			if C(D.internal)!=1:raise E(f"Dyad {D.id}: expected exactly 1 internal joint, got {C(D.internal)}")
			O=D.internal[0];P=O.pt_i;Q=O.pt_j;Z=O.link_i;b=O.link_j
			for q in D.external:
				L=q.pt_i;M=q.pt_j
				if L.x is not A and M.x is A:M.set_global(L.x,L.y)
				elif M.x is not A and L.x is A:L.set_global(M.x,M.y)
			def R(link,ext):
				A=ext
				if A.link_i is link:return A.pt_i
				if A.link_j is link:return A.pt_j
			J=R(Z,o)or R(Z,p);K=R(b,o)or R(b,p)
			if J is A or K is A:return
			if J.x is A or K.x is A:return
			x,y=J.x,J.y;z,A0=K.x,K.y;A1=B.hypot(P.u-J.u,P.v-J.v);A2=B.hypot(Q.u-K.u,Q.v-K.v);r=AF(x,y,A1,z,A0,A2,solution=D.solution)
			if r is A:return
			s,t=r;P.set_global(s,t);Q.set_global(s,t);T(Z,J,P);T(b,K,Q);U=[A for A in D.links if A.ground];V=[A for A in D.links if not A.ground]
class Z(O):
	@G
	def _from_json(gid,data,links,joints):B=joints;A=data;C=[links[A]for A in A.get(Q,[])];D=[B[A]for A in A.get(U,[])];E=[B[A]for A in A.get(j,[])];F=A.get(k,1);return Z(gid,C,internal=D,external=E,solution=F)
	def solve_position(F,theta_crank=A):
		U=b(F.links);AN=set(U);d=b(F.internal);e=b(F.external)
		if C(U)!=4:raise E(f"ClassIV {F.id}: expected 4 links, got {C(U)}")
		if C(d)!=4:raise E(f"ClassIV {F.id}: expected 4 internal joints, got {C(d)}")
		if C(e)!=2:raise E(f"ClassIV {F.id}: expected 2 external joints, got {C(e)}")
		for D in e:
			f=D.pt_i;g=D.pt_j
			if f.x is not A and g.x is A:g.set_global(f.x,f.y)
			elif g.x is not A and f.x is A:f.set_global(g.x,g.y)
		AG=[]
		for D in e:
			S=AJ(D,AN)
			if S is A:raise E(f"ClassIV {F.id}: external joint {D.id} does not touch any group link.")
			h=K(D,S)
			if h is A or h.x is A:raise E(f"ClassIV {F.id}: external joint {D.id} group-side point has no global coords.")
			AG.append((D,S,h))
		M={A.id:0 for A in U}
		for D in d+e:
			if D.link_i.id in M:M[D.link_i.id]+=1
			if D.link_j.id in M:M[D.link_j.id]+=1
		V=A2(M.values())
		if V!=[2,2,3,3]:raise E(f"ClassIV {F.id}: expected degrees [2,2,3,3], got {V}. "+'Degrees: '+R.join([f"{A}:{B}"for(A,B)in M.items()]))
		AO=[A for A in U if M[A.id]==3];A3=[A for A in U if M[A.id]==2];N={}
		for(D,S,h)in AG:
			if S not in AO:raise E(f"ClassIV {F.id}: external joint {D.id} attaches to non-ternary link {S.id}.")
			N[S]=D,h
		if C(N)!=2:raise E(f"ClassIV {F.id}: expected external joints to hit the two ternaries (got {C(N)}).")
		O,P=b(N.keys());Ab,A4=N[O][0],N[O][1];Ac,A5=N[P][0],N[P][1];q=A4.x,A4.y;r=A5.x,A5.y;i={A:[]for A in A3}
		for D in d:
			if D.link_i in i:i[D.link_i].append(D)
			if D.link_j in i:i[D.link_j].append(D)
		for s in A3:
			if C(i[s])!=2:raise E(f"ClassIV {F.id}: binary link {s.id} does not have exactly 2 internal joints inside group.")
		AH=A2(A3,key=lambda lk:lk.id);W=AH[0];X=AH[1]
		def t(link_a,link_b):
			C=link_b;B=link_a
			for A in d:
				if A.link_i is B and A.link_j is C or A.link_j is B and A.link_i is C:return A
		A6=t(O,W);A7=t(P,W);A8=t(O,X);A9=t(P,X)
		if A in[A6,A7,A8,A9]:raise E(f"ClassIV {F.id}: could not identify required internal joints for cut/rem binaries.")
		u=K(A6,O);v=K(A7,P);Q=K(A8,O);Y=K(A9,P);w=K(A6,W);x=K(A7,W);y=K(A8,X);z=K(A9,X);j=A4;AA=A5;Ad=B.hypot(Q.u-j.u,Q.v-j.v);AP=B.hypot(Y.u-AA.u,Y.v-AA.v);AQ=B.hypot(z.u-y.u,z.v-y.v);AR=B.hypot(x.u-w.u,x.v-w.v);AS=Q.u-j.u;AT=Q.v-j.v;A0=F.solution
		if A1(A0,dict):k=1 if n(A0.get('circle',1))>=0 else-1;Z=n(A0.get('phi',0))
		else:k=1 if n(A0)>=0 else-1;Z=0
		def c(phi,commit=L):
			J=AL([O,P,W,X])
			try:
				K,L=AK(phi,AS,AT);C=q[0]+K;D=q[1]+L;E=AF(C,D,AQ,r[0],r[1],AP,solution=k)
				if E is A:return A,A
				F,G=E;H=j;H.set_global(q[0],q[1]);Q.set_global(C,D);M=T(O,H,Q)
				if not M:return A,A
				I=AA;I.set_global(r[0],r[1]);Y.set_global(F,G);N=T(P,I,Y)
				if not N:return A,A
				R=B.hypot(u.x-v.x,u.y-v.y);S=R-AR;return S,(C,D,F,G)
			finally:
				if not commit:AM(J)
		AB=1440;I=[2.*B.pi*A/AB for A in o(AB+1)];V=[]
		for AU in I:AV,_=c(AU);V.append(AV)
		l=[];AC=1e-06
		for G in o(AB):
			AD=V[G];AE=V[G+1]
			if AD is A or AE is A:continue
			if H(AD)<AC:l.append((I[G],I[G]));continue
			if H(AE)<AC:l.append((I[G+1],I[G+1]));continue
			if AD*AE<0:l.append((I[G],I[G+1]));continue
			AW=.5*(I[G]+I[G+1]);AI,_=c(AW)
			if AI is not A and H(AI)<AC:l.append((I[G],I[G+1]))
		def AX(a,b,iters=50):
			if a==b:return a
			D,E=c(a);F,E=c(b)
			if D is A or F is A:return
			for E in o(iters):
				C=.5*(a+b);B,E=c(C)
				if B is A:return
				if H(B)<1e-08:return C
				if D*B<0:b=C;F=B
				else:a=C;D=B
			return .5*(a+b)
		J=[]
		for(AY,s)in l:
			m=AX(AY,s)
			if m is A:continue
			m=m%(2.*B.pi)
			if all(H((m-A+B.pi)%(2*B.pi)-B.pi)>.001 for A in J):J.append(m)
		J.sort();a.messageBox(f"ClassIV {F.id}\ncircle={k}\nroots found = {C(J)}\nroots (deg) = {[round(A*180/B.pi,3)for A in J]}")
		if C(J)==0:raise E(f"ClassIV {F.id}: no closure roots found for circle={k}")
		if Z not in(0,1):raise E(f"ClassIV {F.id}: phi must be 0 or 1 (got {Z})")
		if Z>=C(J):raise E(f"ClassIV {F.id}: requested phi={Z} but only {C(J)} root(s) exist for circle={k}. Roots (rad): {["{:.6f}".format(A)for A in J]}")
		AZ=J[Z];Aa,Ae=c(AZ,commit=p)
		if Aa is A:raise E(f"ClassIV {F.id}: final phi gave invalid configuration.")
		y.set_global(Q.x,Q.y);z.set_global(Y.x,Y.y);T(X,y,z);w.set_global(u.x,u.y);x.set_global(v.x,v.y);T(W,w,x)
O.TYPE_MAP={i:Y,'classIV':Z}
class c:
	def __init__(A,link,joint,theta0):A.link=link;A.joint=joint;A.theta0=F(theta0)
	@G
	def from_json(data,links,joints):
		B=data
		if B is A:return
		C=links[B['link']];D=joints[B['joint']];E=B.get('theta0',P);return c(C,D,E)
class d:
	def __init__(A,link_radius=.5,hole_radius=3.75,link_thickness=.5):A.link_radius=F(link_radius);A.hole_radius=F(hole_radius);A.link_thickness=F(link_thickness)
	@G
	def from_json(raw_geometry):
		A=raw_geometry;B=A.get('link_radius',.5);C=3.75;D=A.get('link_thickness',.5)
		if B<5:a.messageBox('WARNING! Link Radius must be greater than 5')
		return d(B,C,D)
class m:
	def __init__(A,links,joints,groups,geometry,crank=A):A.links=links;A.joints=joints;A.groups=groups;A.crank=crank;A.geometry=geometry
	@classmethod
	def from_json(F,raw):
		B=raw;G=B.get('LINKS',{});H=B.get('JOINTS',{});I=B.get('GROUPS',{});J=B.get('CRANK',A);K=B.get('GEOMETRY',{});L=d.from_json(K);C={A:W.from_json(A,B)for(A,B)in G.items()};D={A:X.from_json(A,B,C)for(A,B)in H.items()};E=[]
		for(M,N)in I.items():P=O.from_json(M,N,C,D);E.append(P)
		Q=c.from_json(J,C,D);return F(C,D,E,L,Q)
	def postion(B,theta_crank):
		for(C,A)in enumerate(B.groups):
			if C==0:A.solve_position(theta_crank=theta_crank)
			else:A.solve_position()
	def generate(A):
		for B in A.links.values():B.generate(A.joints,A.geometry)
	def connect(C):
		global I;D=I.asBuiltJoints
		for E in C.links.values():
			F=J(E,S,A)
			if E.ground and F is not A:F.isGrounded=p
		G=[]
		for B in C.joints.values():G.append(B.pt_j_name)
		H=l(G);K=0
		for L in H:B.makepin(K,H[L],L,C.geometry);K+=1
		for B in C.joints.values():
			Q=B.link_i;R=B.link_j;M=J(Q,S,A);N=J(R,S,A)
			if M is A or N is A:continue
			O=t(B)
			if O is A:continue
			T=adsk.fusion.JointGeometry.createByCurve(O,adsk.fusion.JointKeyPointTypes.CenterKeyPoint);P=D.createInput(M,N,T);P.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection);U=D.add(P);U.name=B.id
def e(points):A=points;B=sum(A for(A,B)in A)/C(A);D=sum(A for(B,A)in A)/C(A);return B,D
def q(points):A=points;C,D=e(A);return A2(A,key=lambda p:B.atan2(p[1]-D,p[0]-C))
def r(p1,p2,cx,cy,radius):
	A=radius;C,D=p1;E,F=p2;L=E-C;M=F-D;G=B.hypot(L,M)
	if G==0:return
	N=L/G;O=M/G;H=-O;I=N;P=(C+E)*.5;Q=(D+F)*.5;R=cx-P;S=cy-Q
	if R*H+S*I>0:J=-H;K=-I
	else:J=H;K=I
	T=C+J*A,D+K*A;U=E+J*A,F+K*A;return T,U
def s(sketch,pts_xy,socket_types,link_radius,hole_radius):
	J=sketch;G=hole_radius;H=J.sketchCurves.sketchCircles;L=J.sketchCurves.sketchLines;K={}
	for((A,C),E)in zip(pts_xy,socket_types):
		E=(E or D).upper();I=adsk.core.Point3D.create(A,C,0);M=H.addByCenterRadius(I,link_radius);K[A,C]=M
		if E==D:H.addByCenterRadius(I,G)
		elif E=='S':F=G/B.sqrt(2.);N=adsk.core.Point3D.create(A+F,C+F,0);O=adsk.core.Point3D.create(A-F,C-F,0);L.addTwoPointRectangle(N,O)
		else:H.addByCenterRadius(I,G)
	return K
def t(joint):
	B=joint;C=A;E=A
	if J(B,'socket_i',D).upper()==D:C=f(B.link_i,B.pt_i)
	if J(B,'socket_j',D).upper()==D:E=f(B.link_j,B.pt_j)
	return E or C
def u(sketch,pts_xy,link_radius,outer_circles):
	H=sketch;h=H.sketchCurves.sketchLines;i=H.sketchCurves.sketchArcs;M=H.sketchPoints;D=q(pts_xy);N,O=e(D);Q=C(D);I={A:[]for A in D}
	for R in o(Q):
		S=D[R];T=D[(R+1)%Q];U=r(S,T,N,O,link_radius)
		if U is A:continue
		(V,W),(X,Y)=U;I[S].append((V,W));I[T].append((X,Y));j=M.add(adsk.core.Point3D.create(V,W,0));k=M.add(adsk.core.Point3D.create(X,Y,0));h.addByTwoPoints(j,k)
	for((E,F),J)in I.items():
		if C(J)<2:continue
		Z=outer_circles.get((E,F))
		if Z is not A:Z.isConstruction=p
		(a,b),(c,d)=J[0],J[1];l=adsk.core.Point3D.create(E,F,0);m=adsk.core.Point3D.create(a,b,0);v=adsk.core.Point3D.create(c,d,0);K=B.atan2(b-F,a-E);f=B.atan2(d-F,c-E);G=B.atan2(B.sin(f-K),B.cos(f-K))
		if G>0:g=G;L=G-2.*B.pi
		else:g=G;L=G+2.*B.pi
		n=N-E;s=O-F
		def t(sweep):A=K+sweep*.5;C=B.cos(A);D=B.sin(A);return C*n+D*s<P
		u=L if t(L)else g;i.addByCenterStartSweep(l,m,u)
def v(component,sketch,thickness):
	D=sketch
	if D.profiles.count==0:return
	B=A;E=P
	for F in D.profiles:
		try:C=F.areaProperties().area
		except:C=P
		if C>E:E=C;B=F
	if B is A:return
	G=component.features.extrudeFeatures;J=adsk.core.ValueInput.createByReal(thickness);H=G.createInput(B,adsk.fusion.FeatureOperations.NewBodyFeatureOperation);H.setDistanceExtent(L,J);I=G.add(H)
	if I.bodies.count>0:return I.bodies.item(0)
def f(link,pt,tol=.001):
	F=J(link,'body',A);G=J(link,S,A)
	if F is A or G is A or pt.x is A or pt.y is A:return
	P,Q=pt.x,pt.y;C=A;D=1e9;I=1e9
	for K in F.edges:
		E=K.geometry
		if not A1(E,adsk.core.Circle3D):continue
		L=E.center;R,T=L.x,L.y;M=E.radius;N=R-P;O=T-Q;B=N*N+O*O
		if B<=tol*tol:
			if B<D or H(B-D)<1e-09 and M<I:D=B;I=M;C=K
	if C is A:return
	return C.createForAssemblyContext(G)
def AF(c1x,c1y,r1,c2x,c2y,r2,solution=1):
	I=solution;C=r1;E=c2x-c1x;F=c2y-c1y;A=B.hypot(E,F)
	if A<1e-12:return
	if A>C+r2+1e-12:return
	if A<H(C-r2)-1e-12:return
	D=(C*C-r2*r2+A*A)/(2.*A);G=C*C-D*D
	if G<0:G=P
	J=B.sqrt(G);K=c1x+D*E/A;L=c1y+D*F/A;M=-F/A;N=E/A;O=K+I*J*M;Q=L+I*J*N;return O,Q
def T(link,p1,p2):
	E,F=p1.u,p1.v;P,Q=p2.u,p2.v;G,I=p1.x,p1.y;R,S=p2.x,p2.y;J=P-E;K=Q-F
	if H(J)<1e-12 and H(K)<1e-12:return L
	T=B.atan2(K,J);M=R-G;N=S-I
	if H(M)<1e-12 and H(N)<1e-12:return L
	U=B.atan2(N,M);O=U-T;C=B.cos(O);D=B.sin(O);V=G-(C*E-D*F);W=I-(D*E+C*F)
	for A in link.pts.values():X=V+C*A.u-D*A.v;Y=W+D*A.u+C*A.v;A.set_global(X,Y)
	return p
def K(J,link):
	if J.link_i is link:return J.pt_i
	if J.link_j is link:return J.pt_j
def AJ(J,group_links_set):
	A=group_links_set
	if J.link_i in A and J.link_j not in A:return J.link_i
	if J.link_j in A and J.link_i not in A:return J.link_j
	if J.link_i in A:return J.link_i
	if J.link_j in A:return J.link_j
def AK(phi,vx,vy):A=B.cos(phi);C=B.sin(phi);return A*vx-C*vy,C*vx+A*vy
def y(p,q):return B.hypot(p[0]-q[0],p[1]-q[1])
def z(group,topo):B=topo;A=[];A.append(f"ClassIV {group.id} topology:");A.append('Links: '+R.join([A.id for A in B[Q]]));A.append('Internal joints: '+R.join([A.id for A in B['internal_joints']]));A.append('External joints: '+R.join([A.id for A in B['external_joints']]));A.append('Degrees (internal only): '+R.join([f"{A.id}:{B["degree_by_link"][A.id]}"for A in B[Q]]));A.append('Ternaries: '+R.join([A.id for A in B['ternaries']]));A.append('Binaries: '+R.join([A.id for A in B['binaries']]));A.append(f"Disconnected binary: {B["disconnected_binary"].id}");A.append(f"Disconnected joint: {B["disconnected_joint"].id}");a.messageBox('\n'.join(A))
def w(link):
	A={}
	for(C,B)in link.pts.items():A[C]=B.x,B.y
	return A
def x(link,snap):
	for(B,(C,D))in snap.items():A=link.pts[B];A.x=C;A.y=D
def AL(links):return{A:w(A)for A in links}
def AM(snaps):
	for(A,B)in snaps.items():x(A,B)
N=adsk.core.Application.get()
a=N.userInterface
def run_with_json(json_path,theta_crank=0):
	global M,I,N,a
	if not N.activeProduct:raise E('No active product. Open a design or assembly first.')
	M=adsk.fusion.Design.cast(N.activeProduct)
	if not M:raise E('No active Fusion design open.')
	I=M.rootComponent
	with open(json_path,'r')as B:C=json.load(B)
	A=m.from_json(C);A.postion(theta_crank=F(theta_crank));A.generate();A.connect();return A,I
def export_all_stl(root_comp,downloads_path):
	global M,N
	if M is A:M=adsk.fusion.Design.cast(N.activeProduct)
	B=M.exportManager;E=root_comp.allOccurrences
	for C in E:F=os.path.join(downloads_path,C.component.name);D=B.createSTLExportOptions(C,F);D.sendToPrintUtility=L;B.execute(D)