%% Suspension Kinematics of Double Wishbone Rack and Pinion for Front Axle
clear;clc
par.umx = 0.08255 /2; % max. rack displacement
par.phmx = 7.55 /2 /180*pi; % max rotation of lower arm
par.rvwk=[0.89604; -0.605; 0.2032]; % W wheel rim center
par.rvak=[0.834; -0.201; 0.104816]; % A lower arm @ chassis rear
par.rvbk=[1.04; -0.201; 0.104816]; % B lower arm @ chassis front
par.rvck=[ 0.90186; -0.5583; 0.121266]; % C lower arm @ wheel body
par.rvdk=[0.8316; -0.2819; 0.2420437]; % D upper arm @ body rear
par.rvek=[1.002; -0.2819; 0.2420437]; % E upper arm @ body front
par.rvfk=[0.88988; -0.5341618; 0.2926]; % F upper arm @ wheel body
par.rvrk=[0.8404; -0.21368; 0.164]; % R drag link @ rack
par.rvqk=[0.80924; -0.49872; 0.1982]; % Q drag link @ wheel body

% additional data
toe0 = 0 /180*pi; % initial toe angle (ISO def)
camb0= 0 /180*pi; % initial camber angle (ISO def)
rs = 0.23707; % steady state tire radius
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
        xp(i,j)=rvpv(1);yp(i,j)=rvpv(2);zp(i,j)=rvpv(3); % contact point
        toe(i,j) = atan2(-eyrv(1), eyrv(2)); % toe angle (+ rot. z-axis)
        camb(i,j)= atan2( eyrv(3), eyrv(2)); % camber angle (+rot. x-axis)
        ddel(i,j)=norm(pd(:,2)); % partial derivative ddel/du
    end
end

%% -----------------------------------------------------------------------
%  FIGURE 1: Wheel Center Paths and Surface Maps
%  -----------------------------------------------------------------------
figure('Name','Suspension Kinematics - Paths and Angles')

n0=round(n/2); m0=round(m/2); rvpk = par.rvwk + rwpk;

% --- Plot 1: Longitudinal-Vertical Plane (XZ) ---
axes('position',[0.05,0.05,0.20,0.90])
hold on, axis equal, grid on
title('Longitudinal Plane (XZ)', 'FontWeight','bold')
xlabel('x  [m]')
ylabel('z  [m]')
h1 = plot(xw(:,m0), zw(:,m0), 'b-',  'LineWidth',1.5);
h2 = plot(xp(:,m0), zp(:,m0), 'b--', 'LineWidth',1.5);
h3 = plot(par.rvwk(1), par.rvwk(3), 'ok', 'MarkerFaceColor','k');
h4 = plot(rvpk(1),    rvpk(3),    'ok', 'MarkerFaceColor','w');
legend([h1,h2,h3,h4], ...
    'Wheel center W', ...
    'Contact point P', ...
    'W (design pos.)', ...
    'P (design pos.)', ...
    'Location','best', 'FontSize',7)

% --- Plot 2: Lateral-Vertical Plane (YZ) ---
axes('position',[0.30,0.05,0.20,0.90])
hold on, axis equal, grid on
title('Lateral Plane (YZ)', 'FontWeight','bold')
xlabel('y  [m]')
ylabel('z  [m]')
h1 = plot(yw(:,m0), zw(:,m0), 'r-',  'LineWidth',1.5);
h2 = plot(yp(:,m0), zp(:,m0), 'r--', 'LineWidth',1.5);
h3 = plot(par.rvwk(2), par.rvwk(3), 'ok', 'MarkerFaceColor','k');
h4 = plot(rvpk(2),    rvpk(3),    'ok', 'MarkerFaceColor','w');
legend([h1,h2,h3,h4], ...
    'Wheel center W', ...
    'Contact point P', ...
    'W (design pos.)', ...
    'P (design pos.)', ...
    'Location','best', 'FontSize',7)

% --- Plot 3: Toe Angle Surface Map ---
axes('position',[0.60,0.05,0.35,0.40]), colormap('white')
surf(u*1000, phi*180/pi, toe*180/pi)
grid on, view(-40,10)
title('Toe Angle', 'FontWeight','bold')
xlabel('Rack displacement  u  [mm]')
ylabel('Lower arm rotation  \phi  [deg]')
zlabel('Toe angle  \delta_{toe}  [deg]')

% --- Plot 4: Camber Angle Surface Map ---
axes('position',[0.60,0.55,0.35,0.40]), colormap('white')
surf(u*1000, phi*180/pi, camb*180/pi)
grid on, view(-40,10)
title('Camber Angle', 'FontWeight','bold')
xlabel('Rack displacement  u  [mm]')
ylabel('Lower arm rotation  \phi  [deg]')
zlabel('Camber angle  \gamma  [deg]')

%% -----------------------------------------------------------------------
%  FIGURE 2: Steering Kinematics
%  -----------------------------------------------------------------------
figure('Name','Steering Kinematics - Ackermann Comparison')

d1 = del(n0,:); d2 =-d1(m:-1:1); % wheel steering angles in design pos.
a=2.7; s=2*par.rvwk(2); d2a=atan2(a*tan(d1),a+s*tan(d1)); % ackermann

% --- Plot 5: Steering Angles vs Rack Displacement ---
axes('position',[0.05,0.55,0.40,0.35])
hold on, grid on
title('Steering Angles vs Rack Displacement', 'FontWeight','bold')
xlabel('Rack displacement  u  [mm]')
ylabel('Steering angle  \delta  [deg]')
plot(u*1000, d1*180/pi, 'b-',  'LineWidth',1.5)
plot(u*1000, d2*180/pi, 'r--', 'LineWidth',1.5)
legend('Left wheel  \delta_1', 'Right wheel  \delta_2', 'Location','best')
yline(0, 'k:', 'HandleVisibility','off')
xline(0, 'k:', 'HandleVisibility','off')

% --- Plot 6: Steering Sensitivity (d-delta/du) ---
axes('position',[0.05,0.10,0.40,0.35])
hold on, grid on
title('Steering Sensitivity', 'FontWeight','bold')
xlabel('Rack displacement  u  [mm]')
ylabel('d\delta / du  [rad/m]')
plot(u*1000, ddel(n0,:), 'm-', 'LineWidth',1.5)
legend('d\delta/du', 'Location','best')
yline(0, 'k:', 'HandleVisibility','off')
xline(0, 'k:', 'HandleVisibility','off')

% --- Plot 7: Ackermann Comparison ---
axes('position',[0.55,0.20,0.40,0.55])
hold on, axis equal, grid on
title('Ackermann Comparison', 'FontWeight','bold')
xlabel('Outer wheel angle  \delta_1  [deg]')
ylabel('Inner wheel angle  \delta_2  [deg]')
plot(d1*180/pi, d2*180/pi,  'b-',  'LineWidth',2.0)
plot(d1*180/pi, d2a*180/pi, 'r--', 'LineWidth',1.5)
legend('Kinematic steering', ...
       ['Ideal Ackermann  (a=',num2str(a),'m)'], ...
       'Location','best')
yline(0, 'k:', 'HandleVisibility','off')
xline(0, 'k:', 'HandleVisibility','off')