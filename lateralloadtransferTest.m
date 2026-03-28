%% LAT LOAD TRANSFER
clear;clc
paramR26;


%% Constant LAT g

function dxdt = eom(~,x)

param;

lat_g = 1;

[~, M] = roll_angle(lat_g,car,front,rear,frontunsprung,rearunsprung);

% roll stiffness
kf = front.k_roll;
kr = rear.k_roll;

% roll damping
cf = front.cs_roll;
cr = rear.cs_roll;

I = 182.24965; % roll inertia

dxdt = zeros(4,1);

% Front roll
dxdt(1) = x(2);
dxdt(2) = -(kf/I)*x(1) - (cf/I)*x(2) + M(1)/I;

% Rear roll
dxdt(3) = x(4);
dxdt(4) = -(kr/I)*x(3) - (cr/I)*x(4) + M(2)/I;

end

tspan = [0 1];
x0 = [0 0 0 0];

[t,x] = ode45(@eom,tspan,x0);

figure(1);
plot(t, x(:,1)*180/pi, 'LineWidth', 1.5)
hold on
plot(t, x(:,3)*180/pi, 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Roll Angle (deg)')
legend('Front roll','Rear roll')
grid on

figure(2);
plot(t, x(:,2)*180/pi, 'LineWidth', 1.5)
hold on
plot(t, x(:,4)*180/pi, 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')
legend('Front roll Velocity','Rear roll Velocity')
grid on

% Geometric Load Transfer

lat_g = 1;

GLTf = ((car.m*weight_distribution/100*9.81)*lat_g*front.RC)/car.track;
GLTr = ((car.m*(1-weight_distribution/100)*9.81)*lat_g*rear.RC)/car.track;

% Elastic Load Transfer 

ELTf_spring = x(:, 1)*front.k_roll;
ELTf_damper = x(:, 2)*front.cs_roll;

ELTr_spring = x(:, 3)*rear.k_roll;
ELTr_damper = x(:, 4)*rear.cs_roll;

GLTf = ones(length(ELTf_damper), 1)*GLTf;
GLTr = ones(length(ELTr_damper), 1)*GLTr;

% Total Load Transfer

TLTf = ELTf_spring+ELTf_damper+GLTf;
TLTr = ELTr_spring+ELTr_damper+GLTr;

% Plotting Load Transfer

figure(3);
plot(t, ELTf_spring, 'LineWidth', 1.5)
hold on
plot(t, ELTf_damper, 'LineWidth', 1.5)
hold on 
plot(t, GLTf, 'LineWidth', 1.5)
hold on 
plot(t, TLTf, 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Load Transfer (N)')
legend('Front Elastic Spring','Front Elastic Damper', 'Front Geometric', ...
    'Front Total Load Transer')
grid on

figure(4);
plot(t, ELTr_spring, 'LineWidth', 1.5)
hold on
plot(t, ELTr_damper, 'LineWidth', 1.5)
hold on 
plot(t, GLTr, 'LineWidth', 1.5)
hold on 
plot(t, TLTr, 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Load Transfer (N)')
legend('Rear Elastic Spring','Rear Elastic Damper', 'Rear Geometric', ...
    'Rear Total Load Transer')
grid on

% Check
Total_LLT = ones(length(ELTf_damper), 1) * ((lat_g*g*car.m*car.cgh)/car.track);

figure(5);
plot(t, TLTf, 'LineWidth', 1.5)
hold on 
plot(t, TLTr, 'LineWidth', 1.5)
hold on
plot(t, TLTf+TLTr,'LineWidth', 1.5)
hold on
plot(t, Total_LLT, 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Load Transfer (N)')
legend('Total Lateral Load Transfer Front', 'Total Lateral Load Transfer Rear', ...
    'Front & Rear Combined Total', 'Total Lateral Load Transfer Car')