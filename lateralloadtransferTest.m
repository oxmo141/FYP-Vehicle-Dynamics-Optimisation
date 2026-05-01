%% LATERAL LOAD TRANSFER ANALYSIS
clear; clc; close all
paramR26;  % This loads car, front, rear, etc.

%% Constants and Parameters
LAT_G = 1.2;  % Lateral acceleration (g)
ROLL_INERTIA = 182.24965;  % Roll inertia (kg*m^2)
TIME_SPAN = [0 1];
INITIAL_STATE = [0 0 0 0];

% Report Start: Lateral Acceleration Constant
fprintf('=== LATERAL LOAD TRANSFER ANALYSIS REPORT ===\n');
fprintf('Constant Lateral Acceleration: %.2f g (%.2f m/s²)\n\n', LAT_G, LAT_G * 9.81);

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
vehicle.wgt_dist = weight_distribution;

%% Compute Steady-State Moments on Each Axle
[~, M_steady] = roll_angle(LAT_G, car, front, rear, frontunsprung, rearunsprung);
moment_front = M_steady(1);  % Front axle moment (N*m)
moment_rear = M_steady(2);   % Rear axle moment (N*m)

% Display moments
fprintf('Front Axle Moment: %.2f N·m\n', moment_front);
fprintf('Rear Axle Moment: %.2f N·m\n', moment_rear);
fprintf('\n');

%% Calculate Natural Frequencies and Damping Ratios
I = ROLL_INERTIA;  % Shared roll inertia

% Front
omega_nf = sqrt(suspension.kf / I);  % Natural frequency (rad/s)
c_cf = 2 * sqrt(suspension.kf * I);  % Critical damping coefficient (N*s/rad or equivalent)
zeta_f = suspension.cf / c_cf;       % Damping ratio (dimensionless)

% Rear
omega_nr = sqrt(suspension.kr / I);  % Natural frequency (rad/s)
c_cr = 2 * sqrt(suspension.kr * I);  % Critical damping coefficient (N*s/rad or equivalent)
zeta_r = suspension.cr / c_cr;       % Damping ratio (dimensionless)

% Display values
fprintf('Front Natural Frequency: %.2f rad/s (%.2f Hz)\n', omega_nf, omega_nf / (2*pi));
fprintf('Front Damping Ratio (zeta): %.3f\n', zeta_f);
fprintf('Rear Natural Frequency: %.2f rad/s (%.2f Hz)\n', omega_nr, omega_nr / (2*pi));
fprintf('Rear Damping Ratio (zeta): %.3f\n', zeta_r);
fprintf('\n');

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

%% Steady-State Angle Validation
roll_steadystate_rad = roll_angle(LAT_G, car,front,rear,frontunsprung,rearunsprung);
roll_steadystate = roll_steadystate_rad * 180/pi;

%% Calculate and Display Steady-State Values and Rise Times
% Steady-state values (in degrees)
fprintf('Front steady-state roll angle: %.4f deg\n', roll_steadystate(1));
fprintf('Rear steady-state roll angle: %.4f deg\n', roll_steadystate(2));

% Convert simulated roll angles to degrees
roll_front_deg = x(:,1) * 180/pi;
roll_rear_deg = x(:,3) * 180/pi;

ss_front = roll_steadystate(1);
ss_rear = roll_steadystate(2);

% Rise time calculation (10% to 90% of steady-state value)
% Assuming monotonic increase to steady-state (positive LAT_G)

% Front rise time
if ss_front > 0
    idx_10f = find(roll_front_deg >= 0.1 * ss_front, 1, 'first');
    idx_90f = find(roll_front_deg >= 0.9 * ss_front, 1, 'first');
    if ~isempty(idx_10f) && ~isempty(idx_90f) && idx_90f > idx_10f
        t10f = t(idx_10f);
        t90f = t(idx_90f);
        rise_time_front = t90f - t10f;
        fprintf('Front rise time (10-90%%): %.4f s\n', rise_time_front);
    else
        fprintf('Front rise time not reliably detected (may not reach 90%% within time span).\n');
    end
else
    fprintf('Front steady-state is non-positive; rise time not applicable.\n');
end

% Rear rise time
if ss_rear > 0
    idx_10r = find(roll_rear_deg >= 0.1 * ss_rear, 1, 'first');
    idx_90r = find(roll_rear_deg >= 0.9 * ss_rear, 1, 'first');
    if ~isempty(idx_10r) && ~isempty(idx_90r) && idx_90r > idx_10r
        t10r = t(idx_10r);
        t90r = t(idx_90r);
        rise_time_rear = t90r - t10r;
        fprintf('Rear rise time (10-90%%): %.4f s\n', rise_time_rear);
    else
        fprintf('Rear rise time not reliably detected (may not reach 90%% within time span).\n');
    end
else
    fprintf('Rear steady-state is non-positive; rise time not applicable.\n');
end
fprintf('\n');

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
hold on;
plot(t, roll_steadystate(1) * ones(size(t)), ...
    'LineStyle', ':', 'LineWidth', 1.5, ...
    'DisplayName', 'Front Steady-State');
hold on;
plot(t, roll_steadystate(2) * ones(size(t)), ...
    'LineStyle', ':', 'LineWidth', 1.5, ...
    'DisplayName', 'Rear Steady-State');
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
