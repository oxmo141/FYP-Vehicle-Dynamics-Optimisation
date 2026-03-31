%% LATERAL LOAD TRANSFER ANALYSIS - IMPROVED VERSION
clear; clc;
paramR26;  % This loads car, front, rear, etc.

%% Constants and Parameters
LAT_G = 1;  % Lateral acceleration (g)
ROLL_INERTIA = 182.24965;  % Roll inertia (kg*m^2)
TIME_SPAN = [0 1];
INITIAL_STATE = [0 0 0 0];

%% Extract and Pre-compute Suspension Parameters
suspension = struct();
suspension.kf = front.k_roll;
suspension.kr = rear.k_roll;
suspension.cf = front.cs_roll;
suspension.cr = rear.cs_roll;
suspension.RC_f = front.RC;
suspension.RC_r = rear.RC;

% Vehicle parameters
vehicle = struct();
vehicle.mass = car.m;
vehicle.track = car.track;
vehicle.cgh = car.cgh;
vehicle.wgt_dist = weight_distribution / 100;

%% Equations of Motion
% Pass car, front, rear as parameters to avoid scope issues
function dxdt = eom(~, x, suspension, vehicle, LAT_G, car, front, rear, frontunsprung, rearunsprung)
    [~, M] = roll_angle(LAT_G, car, front, rear, frontunsprung, rearunsprung);
    
    I = 182.24965;
    dxdt = zeros(4, 1);
    
    % Front roll dynamics
    dxdt(1) = x(2);
    dxdt(2) = -(suspension.kf/I)*x(1) - (suspension.cf/I)*x(2) + M(1)/I;
    
    % Rear roll dynamics
    dxdt(3) = x(4);
    dxdt(4) = -(suspension.kr/I)*x(3) - (suspension.cr/I)*x(4) + M(2)/I;
end

%% Solve ODE
[t, x] = ode45(@(t,x) eom(t, x, suspension, vehicle, LAT_G, car, front, rear, frontunsprung, rearunsprung), ...
    TIME_SPAN, INITIAL_STATE);

%% Pre-calculate Load Transfers (Vectorized)
% Geometric Load Transfer (GLT)
GLT_f = (vehicle.mass * vehicle.wgt_dist * 9.81 * LAT_G * suspension.RC_f) / vehicle.track;
GLT_r = (vehicle.mass * (1 - vehicle.wgt_dist) * 9.81 * LAT_G * suspension.RC_r) / vehicle.track;

% Elastic Load Transfer (ELT) - Spring component
ELT_f_spring = x(:, 1) * suspension.kf;
ELT_r_spring = x(:, 3) * suspension.kr;

% Elastic Load Transfer (ELT) - Damper component
ELT_f_damper = x(:, 2) * suspension.cf;
ELT_r_damper = x(:, 4) * suspension.cr;

% Total Load Transfer (TLT)
n_points = length(t);
GLT_f_vec = GLT_f * ones(n_points, 1);
GLT_r_vec = GLT_r * ones(n_points, 1);

TLT_f = ELT_f_spring + ELT_f_damper + GLT_f_vec;
TLT_r = ELT_r_spring + ELT_r_damper + GLT_r_vec;

% Total lateral load transfer (Check)
Total_LLT = (LAT_G * 9.81 * vehicle.mass * vehicle.cgh / vehicle.track) * ones(n_points, 1);

%% Plotting Function
function plot_load_transfer(t, ELT_spring, ELT_damper, GLT, TLT, location)
    figure;
    plot(t, ELT_spring, 'LineWidth', 1.5, 'DisplayName', 'Elastic Spring');
    hold on;
    plot(t, ELT_damper, 'LineWidth', 1.5, 'DisplayName', 'Elastic Damper');
    plot(t, GLT, 'LineWidth', 1.5, 'DisplayName', 'Geometric');
    plot(t, TLT, 'LineWidth', 1.5, 'DisplayName', 'Total Load Transfer');
    
    xlabel('Time (s)');
    ylabel('Load Transfer (N)');
    title(sprintf('%s Load Transfer Breakdown', location));
    legend('Location', 'best');
    grid on;
end

%% Generate Plots
% Roll angles
figure(1);
plot(t, x(:,1)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Front');
hold on;
plot(t, x(:,3)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Rear');
xlabel('Time (s)');
ylabel('Roll Angle (deg)');
title('Roll Angles vs Time');
legend('Location', 'best');
grid on;

% Angular velocities
figure(2);
plot(t, x(:,2)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Front');
hold on;
plot(t, x(:,4)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Rear');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
title('Roll Angular Velocities vs Time');
legend('Location', 'best');
grid on;

% Load transfer breakdowns
plot_load_transfer(t, ELT_f_spring, ELT_f_damper, GLT_f_vec, TLT_f, 'Front');
plot_load_transfer(t, ELT_r_spring, ELT_r_damper, GLT_r_vec, TLT_r, 'Rear');

% Total load transfer verification
figure(6);
plot(t, TLT_f, 'LineWidth', 1.5, 'DisplayName', 'Front Total');
hold on;
plot(t, TLT_r, 'LineWidth', 1.5, 'DisplayName', 'Rear Total');
plot(t, TLT_f + TLT_r, 'LineWidth', 1.5, 'DisplayName', 'Combined Total');
plot(t, Total_LLT, 'LineWidth', 1.5, 'DisplayName', 'Vehicle Total');
xlabel('Time (s)');
ylabel('Load Transfer (N)');
title('Total Lateral Load Transfer Verification');
legend('Location', 'best');
grid on;
