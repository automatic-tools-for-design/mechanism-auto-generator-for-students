u='solution'
t='external'
o='dyad'
n='plane'
m=property
W='internal'
A4=isinstance
V='occurrence'
S='links'
s=range
r=sorted
q=dict
p=int
Q=.0
e=True
d=set
U=list
R=', '
N='*'
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
from collections import Counter as v
class X:
	def __init__(B,u,v,x=A,y=A):B.u=F(u);B.v=F(v);B.x=A if x is A else F(x);B.y=A if y is A else F(y)
	@G
	def from_json(raw_point):A=raw_point;return X(A[0],A[1])
	def set_global(A,x,y):A.x=F(x);A.y=F(y)
class Y:
	def __init__(B,link_id,ground=L,plane=0,pts=A):B.id=link_id;B.ground=bool(ground);B.plane=p(plane);B.pts=pts if pts is not A else{};B.component=A;B.occurrence=A;B.body=A
	@G
	def from_json(link_id,data):
		A=data;B={};C=A.get('pts',{})
		for(D,E)in C.items():B[D]=X.from_json(E)
		return Y(link_id,ground=A.get('ground',L),plane=A.get(n,0),pts=B)
	def generate(B,joints,geometry):
		Y=joints;P=geometry;global I;Q=[]
		for(h,R)in B.pts.items():
			if R.x is A or R.y is A:continue
			Q.append((h,R.x,R.y))
		if not Q:return
		def H(n):return n[:-1]if n.endswith(N)else n
		S=[A for(A,B,B)in Q];F=[(A,B)for(C,A,B)in Q];T=d()
		for E in Y.values():
			if E.link_i is B:T.add(H(E.pt_i_name))
			if E.link_j is B:T.add(H(E.pt_j_name))
		K={A:D for A in S}
		for E in Y.values():
			if E.link_i is B:
				L=H(E.pt_i_name)
				for M in(L,L+N):
					if M in K:K[M]=(E.socket_i or D).upper()
			if E.link_j is B:
				L=H(E.pt_j_name)
				for M in(L,L+N):
					if M in K:K[M]=(E.socket_j or D).upper()
		l=[K[A]for A in S];m=[P.hole_radius if H(A)in T else P.ref_hole_radius for A in S];V=P.link_radius;Z=P.link_thickness;o=I.occurrences;p=adsk.core.Matrix3D.create();a=o.addNewComponent(p);G=a.component;G.name=f"Link {B.id}";B.component=G;B.occurrence=a;B.body=A;b=G.xYConstructionPlane;c=J(B,n,0)
		if c==0:W=b
		else:e=G.constructionPlanes;r=adsk.core.ValueInput.createByReal(c*Z);f=e.createInput();f.setByOffset(b,r);W=e.add(f)
		O=G.sketches.add(W);O=G.sketches.add(W);X=j(F)
		if C(X)<3:X=i(U(q.fromkeys(F)))
		s=d(X);g=y(O,F,l,V,m,hull_set=s)
		if C(F)>1:k(O,F,V,g)
		if C(F)>1:k(O,F,V,g)
		B.body=A0(G,O,Z)
class Z:
	def __init__(A,joint_id,link_i,pt_i_name,link_j,pt_j_name,socket_i=D,socket_j=D):A.id=joint_id;A.link_i=link_i;A.pt_i_name=pt_i_name;A.link_j=link_j;A.pt_j_name=pt_j_name;A.socket_i=(socket_i or D).upper();A.socket_j=(socket_j or D).upper()
	@m
	def pt_i(self):return self.link_i.pts[self.pt_i_name]
	@m
	def pt_j(self):return self.link_j.pts[self.pt_j_name]
	@G
	def from_json(joint_id,data,links):
		F=links;E=data
		if A4(E,q):G=E.get('ends',[])
		else:G=E
		A,B=G;H=A[0];I=A[1];J=A[2]if C(A)>=3 else D;K=B[0];L=B[1];M=B[2]if C(B)>=3 else D;N=F[H];O=F[K];return Z(joint_id,N,I,O,L,socket_i=J,socket_j=M)
	def makepin(r,idx,num_links,name,geometry):O=num_links;M=geometry;global I;i=I.occurrences;j=adsk.core.Matrix3D.create();k=i.addNewComponent(j);G=k.component;G.name=f"Joint {name}";l=G.xYConstructionPlane;P=G.constructionPlanes;m=adsk.core.ValueInput.createByReal(-2*M.link_thickness);Q=P.createInput();Q.setByOffset(l,m);n=P.add(Q);N=G.sketches.add(n);R=5;E=R;S=1.5;T=1;J=2;H=1;K=1+M.link_thickness*O;F=M.link_thickness*(O+1)+.125;A=-25-10*idx;B=0;C=0;s=adsk.core.Point3D.create(A,B,C);U=adsk.core.Point3D.create(A+E/2,B,C);V=adsk.core.Point3D.create(A-E/2,B,C);W=adsk.core.Point3D.create(A+E/2,B+F,C);X=adsk.core.Point3D.create(A-E/2,B+F,C);Y=adsk.core.Point3D.create(A+E/2+T,B+F,C);Z=adsk.core.Point3D.create(A-E/2-T,B+F,C);a=adsk.core.Point3D.create(A+E/2,B+F+J,C);b=adsk.core.Point3D.create(A-E/2,B+F+J,C);c=adsk.core.Point3D.create(A+E/2-S,B+F+J,C);d=adsk.core.Point3D.create(A-E/2+S,B+F+J,C);o=adsk.core.Point3D.create(A,B+K+H,C);e=adsk.core.Point3D.create(A+H,B+K+H,C);f=adsk.core.Point3D.create(A-H,B+K+H,C);t=adsk.core.Point3D.create(A,B+K,C);D=N.sketchCurves.sketchLines;D.addByTwoPoints(U,W);D.addByTwoPoints(W,Y);D.addByTwoPoints(Y,a);D.addByTwoPoints(a,c);D.addByTwoPoints(c,e);p=N.sketchCurves.sketchArcs;p.addByCenterStartEnd(o,f,e);D.addByTwoPoints(d,f);D.addByTwoPoints(b,d);D.addByTwoPoints(Z,b);D.addByTwoPoints(X,Z);D.addByTwoPoints(V,X);D.addByTwoPoints(V,U);g=G.features.extrudeFeatures;q=adsk.core.ValueInput.createByReal(R);h=g.createInput(N.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation);h.setDistanceExtent(L,q);u=g.add(h)
class P:
	TYPE_MAP={}
	def __init__(B,group_id,links,internal=A,external=A,solution=1):D=external;C=internal;B.id=group_id;B.links=links;B.internal=C if C is not A else[];B.external=D if D is not A else[];B.solution=solution
	@G
	def from_json(group_id,data,links,joints):
		B=group_id;C=data.get('type',o);D=P.TYPE_MAP.get(C)
		if D is A:raise ValueError(f'Unknown group type "{C}" in GROUPS["{B}"]')
		return D._from_json(B,data,links,joints)
	@G
	def _from_json(group_id,data,links,joints):raise NotImplementedError
class a(P):
	@G
	def _from_json(dyad_id,data,links,joints):
		B=joints;A=data;D=[links[A]for A in A.get(S,[])];C=[]
		if W in A:C=[B[A[W]]]
		E=[B[A]for A in A.get(t,[])];F=A.get(u,1);return a(dyad_id,D,internal=C,external=E,solution=F)
	def solve_position(D,theta_crank=A):
		c=theta_crank;U=[A for A in D.links if A.ground];V=[A for A in D.links if not A.ground];u=C(U)==1 and C(V)==1
		if u:
			if c is A:raise E('Driving dyad requires theta_crank.')
			W=U[0];d=V[0];O=D.internal[0];G=O.pt_i_name;H=O.pt_j_name
			if G!=H:
				if C(G)>C(H):G,H=H,G
				if H==G+N:0
				else:raise E(f"Joint {O.id}: mismatched point labels {G} vs {H}")
			if G[-1]==N and H[-1]==N:b.messageBox(f"WARNING: Joint {O.id}- Both joints end in '*' and will not be able to rotate. Please revise")
			X=G
			for I in W.pts.values():I.set_global(I.u,I.v)
			try:Y=W.pts[X]
			except:Y=W.pts[X+N]
			e,f=Y.x,Y.y;g=d.pts[X];h,i=g.u,g.v
			if e!=h or f!=i:raise E(f"cordinates for first dyad do not match")
			j=F(c);k=B.cos(j);l=B.sin(j)
			for I in d.pts.values():m=I.u-h;n=I.v-i;v=e+k*m-l*n;w=f+l*m+k*n;I.set_global(v,w)
		else:
			if C(D.external)!=2:return
			o=D.external[0];p=D.external[1]
			if C(D.internal)!=1:raise E(f"Dyad {D.id}: expected exactly 1 internal joint, got {C(D.internal)}")
			P=D.internal[0];Q=P.pt_i;R=P.pt_j;Z=P.link_i;a=P.link_j
			for q in D.external:
				L=q.pt_i;M=q.pt_j
				if L.x is not A and M.x is A:M.set_global(L.x,L.y)
				elif M.x is not A and L.x is A:L.set_global(M.x,M.y)
			def S(link,ext):
				A=ext
				if A.link_i is link:return A.pt_i
				if A.link_j is link:return A.pt_j
			J=S(Z,o)or S(Z,p);K=S(a,o)or S(a,p)
			if J is A or K is A:return
			if J.x is A or K.x is A:return
			x,y=J.x,J.y;z,A0=K.x,K.y;A1=B.hypot(Q.u-J.u,Q.v-J.v);A2=B.hypot(R.u-K.u,R.v-K.v);r=AH(x,y,A1,z,A0,A2,solution=D.solution)
			if r is A:return
			s,t=r;Q.set_global(s,t);R.set_global(s,t);T(Z,J,Q);T(a,K,R);U=[A for A in D.links if A.ground];V=[A for A in D.links if not A.ground]
class c(P):
	@G
	def _from_json(gid,data,links,joints):B=joints;A=data;C=[links[A]for A in A.get(S,[])];D=[B[A]for A in A.get(W,[])];E=[B[A]for A in A.get(t,[])];F=A.get(u,1);return c(gid,C,internal=D,external=E,solution=F)
	def solve_position(F,theta_crank=A):
		V=U(F.links);AP=d(V);f=U(F.internal);g=U(F.external)
		if C(V)!=4:raise E(f"ClassIV {F.id}: expected 4 links, got {C(V)}")
		if C(f)!=4:raise E(f"ClassIV {F.id}: expected 4 internal joints, got {C(f)}")
		if C(g)!=2:raise E(f"ClassIV {F.id}: expected 2 external joints, got {C(g)}")
		for D in g:
			h=D.pt_i;i=D.pt_j
			if h.x is not A and i.x is A:i.set_global(h.x,h.y)
			elif i.x is not A and h.x is A:h.set_global(i.x,i.y)
		AI=[]
		for D in g:
			S=AL(D,AP)
			if S is A:raise E(f"ClassIV {F.id}: external joint {D.id} does not touch any group link.")
			j=K(D,S)
			if j is A or j.x is A:raise E(f"ClassIV {F.id}: external joint {D.id} group-side point has no global coords.")
			AI.append((D,S,j))
		M={A.id:0 for A in V}
		for D in f+g:
			if D.link_i.id in M:M[D.link_i.id]+=1
			if D.link_j.id in M:M[D.link_j.id]+=1
		W=r(M.values())
		if W!=[2,2,3,3]:raise E(f"ClassIV {F.id}: expected degrees [2,2,3,3], got {W}. "+'Degrees: '+R.join([f"{A}:{B}"for(A,B)in M.items()]))
		AQ=[A for A in V if M[A.id]==3];A5=[A for A in V if M[A.id]==2];N={}
		for(D,S,j)in AI:
			if S not in AQ:raise E(f"ClassIV {F.id}: external joint {D.id} attaches to non-ternary link {S.id}.")
			N[S]=D,j
		if C(N)!=2:raise E(f"ClassIV {F.id}: expected external joints to hit the two ternaries (got {C(N)}).")
		O,P=U(N.keys());Ad,A6=N[O][0],N[O][1];Ae,A7=N[P][0],N[P][1];t=A6.x,A6.y;u=A7.x,A7.y;k={A:[]for A in A5}
		for D in f:
			if D.link_i in k:k[D.link_i].append(D)
			if D.link_j in k:k[D.link_j].append(D)
		for v in A5:
			if C(k[v])!=2:raise E(f"ClassIV {F.id}: binary link {v.id} does not have exactly 2 internal joints inside group.")
		AJ=r(A5,key=lambda lk:lk.id);X=AJ[0];Y=AJ[1]
		def w(link_a,link_b):
			C=link_b;B=link_a
			for A in f:
				if A.link_i is B and A.link_j is C or A.link_j is B and A.link_i is C:return A
		A8=w(O,X);A9=w(P,X);AA=w(O,Y);AB=w(P,Y)
		if A in[A8,A9,AA,AB]:raise E(f"ClassIV {F.id}: could not identify required internal joints for cut/rem binaries.")
		x=K(A8,O);y=K(A9,P);Q=K(AA,O);Z=K(AB,P);z=K(A8,X);A0=K(A9,X);A1=K(AA,Y);A2=K(AB,Y);l=A6;AC=A7;Af=B.hypot(Q.u-l.u,Q.v-l.v);AR=B.hypot(Z.u-AC.u,Z.v-AC.v);AS=B.hypot(A2.u-A1.u,A2.v-A1.v);AT=B.hypot(A0.u-z.u,A0.v-z.v);AU=Q.u-l.u;AV=Q.v-l.v;A3=F.solution
		if A4(A3,q):m=1 if p(A3.get('circle',1))>=0 else-1;a=p(A3.get('phi',0))
		else:m=1 if p(A3)>=0 else-1;a=0
		def c(phi,commit=L):
			J=AN([O,P,X,Y])
			try:
				K,L=AM(phi,AU,AV);C=t[0]+K;D=t[1]+L;E=AH(C,D,AS,u[0],u[1],AR,solution=m)
				if E is A:return A,A
				F,G=E;H=l;H.set_global(t[0],t[1]);Q.set_global(C,D);M=T(O,H,Q)
				if not M:return A,A
				I=AC;I.set_global(u[0],u[1]);Z.set_global(F,G);N=T(P,I,Z)
				if not N:return A,A
				R=B.hypot(x.x-y.x,x.y-y.y);S=R-AT;return S,(C,D,F,G)
			finally:
				if not commit:AO(J)
		AD=1440;I=[2.*B.pi*A/AD for A in s(AD+1)];W=[]
		for AW in I:AX,_=c(AW);W.append(AX)
		n=[];AE=1e-06
		for G in s(AD):
			AF=W[G];AG=W[G+1]
			if AF is A or AG is A:continue
			if H(AF)<AE:n.append((I[G],I[G]));continue
			if H(AG)<AE:n.append((I[G+1],I[G+1]));continue
			if AF*AG<0:n.append((I[G],I[G+1]));continue
			AY=.5*(I[G]+I[G+1]);AK,_=c(AY)
			if AK is not A and H(AK)<AE:n.append((I[G],I[G+1]))
		def AZ(a,b,iters=50):
			if a==b:return a
			D,E=c(a);F,E=c(b)
			if D is A or F is A:return
			for E in s(iters):
				C=.5*(a+b);B,E=c(C)
				if B is A:return
				if H(B)<1e-08:return C
				if D*B<0:b=C;F=B
				else:a=C;D=B
			return .5*(a+b)
		J=[]
		for(Aa,v)in n:
			o=AZ(Aa,v)
			if o is A:continue
			o=o%(2.*B.pi)
			if all(H((o-A+B.pi)%(2*B.pi)-B.pi)>.001 for A in J):J.append(o)
		J.sort();b.messageBox(f"ClassIV {F.id}\ncircle={m}\nroots found = {C(J)}\nroots (deg) = {[round(A*180/B.pi,3)for A in J]}")
		if C(J)==0:raise E(f"ClassIV {F.id}: no closure roots found for circle={m}")
		if a not in(0,1):raise E(f"ClassIV {F.id}: phi must be 0 or 1 (got {a})")
		if a>=C(J):raise E(f"ClassIV {F.id}: requested phi={a} but only {C(J)} root(s) exist for circle={m}. Roots (rad): {["{:.6f}".format(A)for A in J]}")
		Ab=J[a];Ac,Ag=c(Ab,commit=e)
		if Ac is A:raise E(f"ClassIV {F.id}: final phi gave invalid configuration.")
		A1.set_global(Q.x,Q.y);A2.set_global(Z.x,Z.y);T(Y,A1,A2);z.set_global(x.x,x.y);A0.set_global(y.x,y.y);T(X,z,A0)
P.TYPE_MAP={o:a,'classIV':c}
class f:
	def __init__(A,link,joint,theta0):A.link=link;A.joint=joint;A.theta0=F(theta0)
	@G
	def from_json(data,links,joints):
		B=data
		if B is A:return
		C=links[B['link']];D=joints[B['joint']];E=B.get('theta0',Q);return f(C,D,E)
class g:
	def __init__(A,link_radius=.5,hole_radius=3.75,ref_hole_radius=1.5,link_thickness=.5):A.link_radius=F(link_radius);A.hole_radius=F(hole_radius);A.ref_hole_radius=F(ref_hole_radius);A.link_thickness=F(link_thickness)
	@G
	def from_json(raw_geometry):
		A=raw_geometry;B=A.get('link_radius',.5);C=A.get('hole_radius',3.75);D=A.get('ref_hole_radius',1.5);E=A.get('link_thickness',.5)
		if B<5:b.messageBox('WARNING! Link Radius must be greater than 5')
		return g(B,C,D,E)
class w:
	def __init__(A,links,joints,groups,geometry,crank=A):A.links=links;A.joints=joints;A.groups=groups;A.crank=crank;A.geometry=geometry
	@classmethod
	def from_json(F,raw):
		B=raw;G=B.get('LINKS',{});H=B.get('JOINTS',{});I=B.get('GROUPS',{});J=B.get('CRANK',A);K=B.get('GEOMETRY',{});L=g.from_json(K);C={A:Y.from_json(A,B)for(A,B)in G.items()};D={A:Z.from_json(A,B,C)for(A,B)in H.items()};E=[]
		for(M,N)in I.items():O=P.from_json(M,N,C,D);E.append(O)
		Q=f.from_json(J,C,D);return F(C,D,E,L,Q)
	def postion(B,theta_crank):
		for(C,A)in enumerate(B.groups):
			if C==0:A.solve_position(theta_crank=theta_crank)
			else:A.solve_position()
	def generate(A):
		for B in A.links.values():B.generate(A.joints,A.geometry)
	def connect(C):
		global I;D=I.asBuiltJoints
		for E in C.links.values():
			F=J(E,V,A)
			if E.ground and F is not A:F.isGrounded=e
		G=[]
		for B in C.joints.values():G.append(B.pt_j_name)
		H=v(G);K=0
		for L in H:B.makepin(K,H[L],L,C.geometry);K+=1
		for B in C.joints.values():
			Q=B.link_i;R=B.link_j;M=J(Q,V,A);N=J(R,V,A)
			if M is A or N is A:continue
			O=z(B)
			if O is A:continue
			S=adsk.fusion.JointGeometry.createByCurve(O,adsk.fusion.JointKeyPointTypes.CenterKeyPoint);P=D.createInput(M,N,S);P.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection);T=D.add(P);T.name=B.id
def h(points):A=points;B=sum(A for(A,B)in A)/C(A);D=sum(A for(B,A)in A)/C(A);return B,D
def i(points):A=points;C,D=h(A);return r(A,key=lambda p:B.atan2(p[1]-D,p[0]-C))
def x(p1,p2,cx,cy,radius):
	A=radius;C,D=p1;E,F=p2;L=E-C;M=F-D;G=B.hypot(L,M)
	if G==0:return
	N=L/G;O=M/G;H=-O;I=N;P=(C+E)*.5;Q=(D+F)*.5;R=cx-P;S=cy-Q
	if R*H+S*I>0:J=-H;K=-I
	else:J=H;K=I
	T=C+J*A,D+K*A;U=E+J*A,F+K*A;return T,U
def y(sketch,pts_xy,socket_types,link_radius,hole_radii,hull_set=A):
	I=sketch;F=hull_set;G=I.sketchCurves.sketchCircles;M=I.sketchCurves.sketchLines;J={};F=F or d()
	for((A,B),C,K)in zip(pts_xy,socket_types,hole_radii):
		C=(C or D).upper();H=adsk.core.Point3D.create(A,B,0);L=G.addByCenterRadius(H,link_radius)
		if(A,B)not in F:L.isConstruction=e
		J[A,B]=L
		if C==D:G.addByCenterRadius(H,K)
		elif C=='S':E=5.125/2;N=adsk.core.Point3D.create(A+E,B+E,0);O=adsk.core.Point3D.create(A-E,B-E,0);M.addTwoPointRectangle(N,O)
		else:G.addByCenterRadius(H,K)
	return J
def z(joint):
	B=joint;C=A;E=A
	if J(B,'socket_i',D).upper()==D:C=l(B.link_i,B.pt_i)
	if J(B,'socket_j',D).upper()==D:E=l(B.link_j,B.pt_j)
	return E or C
def j(points,eps=1e-09):
	E=r(d(points))
	if C(E)<=1:return E
	def F(o,a,b):return(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
	A=[]
	for D in E:
		while C(A)>=2 and F(A[-2],A[-1],D)<=eps:A.pop()
		A.append(D)
	B=[]
	for D in reversed(E):
		while C(B)>=2 and F(B[-2],B[-1],D)<=eps:B.pop()
		B.append(D)
	return A[:-1]+B[:-1]
def k(sketch,pts_xy,link_radius,outer_circles):
	I=pts_xy;H=sketch;k=H.sketchCurves.sketchLines;l=H.sketchCurves.sketchArcs;N=H.sketchPoints
	if C(I)<2:return[]
	D=j(I)
	if C(D)<3:D=i(U(q.fromkeys(I)))
	if C(D)<2:return[]
	O,P=h(D);R=C(D);J={A:[]for A in D}
	for S in s(R):
		T=D[S];V=D[(S+1)%R];W=x(T,V,O,P,link_radius)
		if W is A:continue
		(X,Y),(Z,a)=W;J[T].append((X,Y));J[V].append((Z,a));m=N.add(adsk.core.Point3D.create(X,Y,0));n=N.add(adsk.core.Point3D.create(Z,a,0));k.addByTwoPoints(m,n)
	for((E,F),K)in J.items():
		if C(K)<2:continue
		b=outer_circles.get((E,F))
		if b is not A:b.isConstruction=e
		(c,d),(o,p)=K[0],K[1];r=adsk.core.Point3D.create(E,F,0);t=adsk.core.Point3D.create(c,d,0);L=B.atan2(d-F,c-E);f=B.atan2(p-F,o-E);G=B.atan2(B.sin(f-L),B.cos(f-L))
		if G>0:g=G;M=G-2.*B.pi
		else:g=G;M=G+2.*B.pi
		u=O-E;v=P-F
		def w(sweep):A=L+.5*sweep;C=B.cos(A);D=B.sin(A);return C*u+D*v<Q
		y=M if w(M)else g;l.addByCenterStartSweep(r,t,y)
	return D
def A0(component,sketch,thickness):
	D=sketch
	if D.profiles.count==0:return
	B=A;E=Q
	for F in D.profiles:
		try:C=F.areaProperties().area
		except:C=Q
		if C>E:E=C;B=F
	if B is A:return
	G=component.features.extrudeFeatures;J=adsk.core.ValueInput.createByReal(thickness);H=G.createInput(B,adsk.fusion.FeatureOperations.NewBodyFeatureOperation);H.setDistanceExtent(L,J);I=G.add(H)
	if I.bodies.count>0:return I.bodies.item(0)
def l(link,pt,tol=.001):
	F=J(link,'body',A);G=J(link,V,A)
	if F is A or G is A or pt.x is A or pt.y is A:return
	P,Q=pt.x,pt.y;C=A;D=1e9;I=1e9
	for K in F.edges:
		E=K.geometry
		if not A4(E,adsk.core.Circle3D):continue
		L=E.center;R,S=L.x,L.y;M=E.radius;N=R-P;O=S-Q;B=N*N+O*O
		if B<=tol*tol:
			if B<D or H(B-D)<1e-09 and M<I:D=B;I=M;C=K
	if C is A:return
	return C.createForAssemblyContext(G)
def AH(c1x,c1y,r1,c2x,c2y,r2,solution=1):
	I=solution;C=r1;E=c2x-c1x;F=c2y-c1y;A=B.hypot(E,F)
	if A<1e-12:return
	if A>C+r2+1e-12:return
	if A<H(C-r2)-1e-12:return
	D=(C*C-r2*r2+A*A)/(2.*A);G=C*C-D*D
	if G<0:G=Q
	J=B.sqrt(G);K=c1x+D*E/A;L=c1y+D*F/A;M=-F/A;N=E/A;O=K+I*J*M;P=L+I*J*N;return O,P
def T(link,p1,p2):
	E,F=p1.u,p1.v;P,Q=p2.u,p2.v;G,I=p1.x,p1.y;R,S=p2.x,p2.y;J=P-E;K=Q-F
	if H(J)<1e-12 and H(K)<1e-12:return L
	T=B.atan2(K,J);M=R-G;N=S-I
	if H(M)<1e-12 and H(N)<1e-12:return L
	U=B.atan2(N,M);O=U-T;C=B.cos(O);D=B.sin(O);V=G-(C*E-D*F);W=I-(D*E+C*F)
	for A in link.pts.values():X=V+C*A.u-D*A.v;Y=W+D*A.u+C*A.v;A.set_global(X,Y)
	return e
def K(J,link):
	if J.link_i is link:return J.pt_i
	if J.link_j is link:return J.pt_j
def AL(J,group_links_set):
	A=group_links_set
	if J.link_i in A and J.link_j not in A:return J.link_i
	if J.link_j in A and J.link_i not in A:return J.link_j
	if J.link_i in A:return J.link_i
	if J.link_j in A:return J.link_j
def AM(phi,vx,vy):A=B.cos(phi);C=B.sin(phi);return A*vx-C*vy,C*vx+A*vy
def A3(p,q):return B.hypot(p[0]-q[0],p[1]-q[1])
def A5(group,topo):B=topo;A=[];A.append(f"ClassIV {group.id} topology:");A.append('Links: '+R.join([A.id for A in B[S]]));A.append('Internal joints: '+R.join([A.id for A in B['internal_joints']]));A.append('External joints: '+R.join([A.id for A in B['external_joints']]));A.append('Degrees (internal only): '+R.join([f"{A.id}:{B["degree_by_link"][A.id]}"for A in B[S]]));A.append('Ternaries: '+R.join([A.id for A in B['ternaries']]));A.append('Binaries: '+R.join([A.id for A in B['binaries']]));A.append(f"Disconnected binary: {B["disconnected_binary"].id}");A.append(f"Disconnected joint: {B["disconnected_joint"].id}");b.messageBox('\n'.join(A))
def A1(link):
	A={}
	for(C,B)in link.pts.items():A[C]=B.x,B.y
	return A
def A2(link,snap):
	for(B,(C,D))in snap.items():A=link.pts[B];A.x=C;A.y=D
def AN(links):return{A:A1(A)for A in links}
def AO(snaps):
	for(A,B)in snaps.items():A2(A,B)
O=adsk.core.Application.get()
b=O.userInterface
def run_with_json(json_path,theta_crank=0):
	global M,I,O,b
	if not O.activeProduct:raise E('No active product. Open a design or assembly first.')
	M=adsk.fusion.Design.cast(O.activeProduct)
	if not M:raise E('No active Fusion design open.')
	I=M.rootComponent
	with open(json_path,'r')as B:C=json.load(B)
	A=w.from_json(C);A.postion(theta_crank=F(theta_crank));A.generate();A.connect();return A,I
def export_all_stl(root_comp,downloads_path):
	global M,O
	if M is A:M=adsk.fusion.Design.cast(O.activeProduct)
	B=M.exportManager;E=root_comp.allOccurrences
	for C in E:F=os.path.join(downloads_path,C.component.name);D=B.createSTLExportOptions(C,F);D.sendToPrintUtility=L;B.execute(D)