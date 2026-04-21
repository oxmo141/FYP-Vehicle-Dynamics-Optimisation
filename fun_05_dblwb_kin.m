function [ avw, rvwv, del, pv ] = fun_05_dblwb_kin( phi, u, p )
% rotation matrix wheel rim / ref-sys
% actual position of wheel center
% rotation angle arround king pin
% partial velocities
% kinematics of a double wishbone suspension
% rotation angle of lower control arm
% rack displacement
% structure of model parameter

% lower wishbone
rab = p.rvbk-p.rvak; eab = rab/norm(rab); eabs = uty_skewsym(eab);
eabeab = eab*eab.'; eyeab = eye(3,3)-eabeab;
aphi = eabeab + eyeab*cos(phi) + eabs*sin(phi);

% upper wishbone
rcfk = p.rvfk-p.rvck; rdfk = p.rvfk-p.rvdk;
racv=aphi*(p.rvck-p.rvak); rcdv=p.rvdk-(p.rvak+racv);
rde=p.rvek-p.rvdk;
ede=rde/norm(rde); edeede=ede*ede.'; edes=uty_skewsym(ede);
a = rcdv.'*(eye(3,3)-edeede)*rdfk; b = rcdv.'*cross(ede,rdfk);
c =-rcdv.'*edeede*rdfk-0.5*(rdfk.'*rdfk+rcdv.'*rcdv-rcfk.'*rcfk);
psi = uty_trigon(a,b,c);
apsi = edeede + (eye(3,3)-edeede)*cos(psi) + edes*sin(psi);

% orientation of wheel body due to control arm motion
rvcv = p.rvak + racv;
rdfv = apsi*(p.rvfk-p.rvdk); rvfv = p.rvdk + rdfv; rcfv = rvfv-rvcv;
be = uty_trigon(rcfk(1),rcfk(3),rcfv(1));
abe = [ cos(be) 0 sin(be) ; ...
    0 1 0 ; ...
    -sin(be) 0 cos(be) ];
al = uty_trigon(rcfv(2),rcfv(3),rcfk(2));
aal = [ 1 0 0 ; ...
    0 cos(al) -sin(al) ; ...
    0 sin(al) cos(al) ] ;

% rotation arround king pin
rvrv = p.rvrk + [ 0; u; 0 ]; rrcv = rvcv-rvrv; rrcht=rrcv.'*aal*abe;
rrqk = p.rvqk-p.rvrk; rcqk = p.rvqk-p.rvck; rcfk = p.rvfk-p.rvck;
ecf = rcfk/norm(rcfk); ecfecf = ecf*ecf.'; ecfs = uty_skewsym(ecf);
a = rrcht*(eye(3,3)-ecfecf)*rcqk; b = rrcht*cross(ecf,rcqk);
c =-(rrcht*ecfecf*rcqk+0.5*(rrcv.'*rrcv+rcqk.'*rcqk-rrqk.'*rrqk));
del=uty_trigon(a,b,c);
adel=ecfecf+(eye(3,3)-ecfecf)*cos(del)+ecfs*sin(del);

% wheel body orientation and position
avw = aal*abe*adel; rcwv = avw*(p.rvwk-p.rvck); rvwv = rvcv + rcwv;

% partial derivatives: dpsi/dphi, dal/dphi, dbe/dphi
exvv=[1;0;0]; eyalv=aal*[0;1;0];
a = [ cross(ede,rdfv)-cross(exvv,rcfv)-cross(eyalv,rcfv) ];
c = a \ cross(eab,racv); dpsidphi=c(1); daldphi=c(2); dbedphi=c(3);

% partial derivatives: ddel/dphi, ddel/du
eyvv=[0;1;0]; ecfv=rcfv/norm(rcfv);
rcqv=avw*rcqk; rvqv=rvcv+rcqv; rrqv=rvqv-rvrv;
a = exvv*daldphi + eyalv*dbedphi; b = cross(eab,racv)+cross(a,rcqv);
c = rrqv.'*cross(ecfv,rcqv);
ddeldphi =-rrqv.'*b/c; ddeldu = rrqv.'*eyvv/c;

% partial angular and partial velocities of wheel carrier
dodphi=exvv*daldphi+eyalv*dbedphi+ecfv*ddeldphi; dodu=ecfv*ddeldu;
dvdphi=cross(eab,racv)+cross(dodphi,rcwv); dvdu = cross(dodu,rcwv);
pv = [ dodphi dodu dvdphi dvdu ];

end