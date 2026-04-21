%% Suspension Kinematics of Double Wishbone Rack and Pinion for Front Axle
par.umx = 0.0745; % max. rack displacement
par.phmx = 10/180*pi; % max rotation of lower arm
par.rvwk=[ 0.000; 0.768; 0.000]; % W wheel rim center
par.rvak=[-0.251; 0.320;-0.080]; % A lower arm @ chassis rear
par.rvbk=[ 0.148; 0.320;-0.094]; % B lower arm @ chassis front
par.rvck=[ 0.013; 0.737;-0.145]; % C lower arm @ wheel body
par.rvdk=[-0.105; 0.435; 0.196]; % D upper arm @ body rear
par.rvek=[ 0.122; 0.435; 0.230]; % E upper arm @ body front
par.rvfk=[-0.025; 0.680; 0.162]; % F upper arm @ wheel body
par.rvrk=[-0.150; 0.380;-0.038]; % R drag link @ rack
par.rvqk=[-0.137; 0.690;-0.088]; % Q drag link @ wheel body

% additional data
toe0 = 0.0000/180*pi; % initial toe angle (ISO def)
camb0= 0.8000/180*pi; % initial camber angle (ISO def)
rs = 0.2850; % steady state tire radius
en0 = [ 0; 0; 1 ]; % road normal (flat horizontal road)

% wheel/tire orientation in design position
eyrk = [toe0; 1;-camb0]; eyrk = eyrk/norm(eyrk); % wheel rot. axis
exk = cross(eyrk,en0); exk=exk/norm(exk); % longitudinal direction
eyk = cross(en0,exk); % lateral direction
ezk = cross(exk,eyrk); % radial direction
rwpk=-rs*ezk; % wheel center W--> P (contact in design pos.)

% kingpin and caster angle in design position
rcfk = par.rvfk-par.rvck; ecfk=rcfk/norm(rcfk); % kingpin orientation
si=atan2(-ecfk(2),ecfk(3)); disp(['sigma = ',num2str(si*180/pi)])
nu=atan2(-ecfk(1),ecfk(3)); disp(['nue = ',num2str(nu*180/pi)])

% caster offset and scrub radius in design position
rcpk = par.rvwk+rwpk-par.rvck; rsck =-(en0.'*rcpk)/(en0.'*ecfk)*ecfk;
co =-exk'*(rsck+rcpk); disp(['caster offset= ',num2str(co)])
sr = eyk'*(rsck+rcpk); disp(['scrub radius= ',num2str(sr)])

% range of motion (rotation of lower arm and rack displacement)
n=11; phi=linspace(-1,1,n)*par.phmx; m=15; u=linspace(-1,1,m)*par.umx;

% pre-allocate vars to speed up loop and compute suspension kinematics
xw=zeros(n,m);yw=xw;zw=xw; xp=xw;yp=xw;zp=xw;
del=xw; toe=xw; camb=xw; ddel=xw;
for i=1:n
    for j=1:m
        [avw,rvwv,del(i,j),pd] = fun_05_dblwb_kin(phi(i),u(j),par);
        eyrv = avw*eyrk; % actual orientation of wheel rotation axis
        rvpv = rvwv + avw*rwpk; % actual position of contact point P
        xw(i,j)=rvwv(1);yw(i,j)=rvwv(2);zw(i,j)=rvwv(3); % wheel center
        xp(i,j)=rvpv(1);yp(i,j)=rvpv(2);zp(i,j)=rvpv(3); % ref. point
        toe(i,j) = atan2(-eyrv(1), eyrv(2)); % toe angle (+ rot. z-axis)
        camb(i,j)= atan2( eyrv(3), eyrv(2)); % camber angle (+rot. x-axis)
        ddel(i,j)=norm(pd(:,2)); % partial derivative ddel/du
    end
end

n0=round(n/2); m0=round(m/2); rvpk = par.rvwk + rwpk ;
axes('position',[0.05,0.05,0.20,0.90])
hold on, axis equal,grid on,title('xz')
plot(xw(:,m0),zw(:,m0)),plot(xp(:,m0),zp(:,m0),'--')
plot(par.rvwk(1),par.rvwk(3),'ok'), plot(rvpk(1),rvpk(3),'ok')
axes('position',[0.30,0.05,0.20,0.90])
hold on, axis equal, grid on, title('yz')
plot(yw(:,m0),zw(:,m0)), plot(yp(:,m0),zp(:,m0),'--')
plot(par.rvwk(2),par.rvwk(3),'ok'),plot(rvpk(2),rvpk(3),'ok')
axes('position',[0.60,0.05,0.35,0.40]), colormap('white')
surf(u,phi*180/pi,toe*180/pi), grid on, view(-40,10), title('toe')
axes('position',[0.60,0.55,0.35,0.40]), colormap('white')
surf(u,phi*180/pi,camb*180/pi), grid on, view(-40,10), title('camber')

figure
d1 = del(n0,:); d2 =-d1(m:-1:1); % wheel steering angles in des. pos.
a=2.7; s=2*par.rvwk(2); d2a=atan2(a*tan(d1),a+s*tan(d1)); % ackermann
axes('position',[0.05,0.55,0.40,0.30]), title('d1(u), d2(u)')
plot(u,[d1;d2]*180/pi), grid on, legend('left','right')
axes('position',[0.05,0.15,0.40,0.30]), title('d\delta/du')
plot(u,ddel(n0,:)), grid on
axes('position',[0.55,0.30,0.40,0.40]), axis equal, title('d2(d1)')
plot(d1*180/pi,[d2;d2a]*180/pi),grid on, legend('kin','ackermann')