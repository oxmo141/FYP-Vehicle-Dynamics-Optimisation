%% LATERAL LOAD TRANSFER ANALYSIS
clear; clc; close all;
paramR26;

% =========================================================================
%% CONSTANTS & PARAMETERS
% =========================================================================
LAT_G        = 1;          % Lateral acceleration (g)
ROLL_INERTIA = 182.24965;  % Roll inertia (kg·m²)
TIME_SPAN    = [0 1];      % Simulation time (s)
INITIAL_STATE = [0 0 0 0]; % [phi_f, dphi_f, phi_r, dphi_r]

fprintf('=== LATERAL LOAD TRANSFER ANALYSIS REPORT ===\n');
fprintf('Lateral Acceleration: %.2f g (%.2f m/s²)\n\n', LAT_G, LAT_G * 9.81);

% =========================================================================
%% SUSPENSION & VEHICLE STRUCTS
% =========================================================================
suspension = struct( ...
    'kf',   front.k_roll, ...
    'kr',   rear.k_roll,  ...
    'cf',   front.cs_roll, ...
    'cr',   rear.cs_roll,  ...
    'RC_f', front.RC,     ...
    'RC_r', rear.RC);

vehicle = struct( ...
    'mass',     car.m,              ...
    'track',    car.track,          ...
    'cgh',      car.cgh,            ...
    'wgt_dist', weight_distribution, ...
    'MR_f',     front.MR,           ...
    'MR_r',     rear.MR);

% =========================================================================
%% STEADY-STATE MOMENTS
% =========================================================================
[~, M_steady] = roll_angle(LAT_G, car, front, rear, frontunsprung, rearunsprung);
moment_front = M_steady(1);
moment_rear  = M_steady(2);

fprintf('Front Axle Moment : %.2f N·m\n', moment_front);
fprintf('Rear  Axle Moment : %.2f N·m\n\n', moment_rear);

% =========================================================================
%% NATURAL FREQUENCIES & DAMPING RATIOS
% =========================================================================
I = ROLL_INERTIA;

omega_nf = sqrt(suspension.kf / I);
zeta_f   = suspension.cf / (2 * sqrt(suspension.kf * I));

omega_nr = sqrt(suspension.kr / I);
zeta_r   = suspension.cr / (2 * sqrt(suspension.kr * I));

fprintf('Front | omega_n: %6.2f rad/s (%5.2f Hz) | zeta: %.3f\n', omega_nf, omega_nf/(2*pi), zeta_f);
fprintf('Rear  | omega_n: %6.2f rad/s (%5.2f Hz) | zeta: %.3f\n\n', omega_nr, omega_nr/(2*pi), zeta_r);

% =========================================================================
%% ODE SOLVER
% =========================================================================
[t, x] = ode45( ...
    @(t, x) eom(t, x, suspension, I, LAT_G, car, front, rear, frontunsprung, rearunsprung), ...
    TIME_SPAN, INITIAL_STATE);

% State vector columns: [phi_f | dphi_f | phi_r | dphi_r]
roll_front_deg = x(:,1) * 180/pi;
roll_rear_deg  = x(:,3) * 180/pi;
omega_front    = x(:,2);           % rad/s
omega_rear     = x(:,4);           % rad/s

% =========================================================================
%% STEADY-STATE ROLL ANGLES
% =========================================================================
roll_ss_rad = roll_angle(LAT_G, car, front, rear, frontunsprung, rearunsprung);
roll_ss     = roll_ss_rad * 180/pi;

fprintf('Front steady-state roll angle: %.4f deg\n',   roll_ss(1));
fprintf('Rear  steady-state roll angle: %.4f deg\n\n', roll_ss(2));

% =========================================================================
%% RISE TIMES (10% → 90%)
% =========================================================================
compute_rise_time(t, roll_front_deg, roll_ss(1), 'Front');
compute_rise_time(t, roll_rear_deg,  roll_ss(2), 'Rear');
fprintf('\n');

% =========================================================================
%% LOAD TRANSFER CALCULATIONS
% =========================================================================
n = length(t);

% Geometric Load Transfer (constant)
GLT_f = (vehicle.mass * vehicle.wgt_dist       * 9.81 * LAT_G * suspension.RC_f) / vehicle.track;
GLT_r = (vehicle.mass * (1 - vehicle.wgt_dist) * 9.81 * LAT_G * suspension.RC_r) / vehicle.track;

GLT_f_vec = GLT_f * ones(n, 1);
GLT_r_vec = GLT_r * ones(n, 1);

% Elastic Load Transfer — spring and damper components
ELT_f_spring = x(:,1) * suspension.kf;
ELT_f_damper = x(:,2) * suspension.cf;
ELT_r_spring = x(:,3) * suspension.kr;
ELT_r_damper = x(:,4) * suspension.cr;

% Total Load Transfer
TLT_f = ELT_f_spring + ELT_f_damper + GLT_f_vec;
TLT_r = ELT_r_spring + ELT_r_damper + GLT_r_vec;

% Vehicle-level LLT check
Total_LLT = (LAT_G * 9.81 * vehicle.mass * vehicle.cgh / vehicle.track) * ones(n, 1);

% Damper linear velocity (mm/s)
v_damper_f = omega_front * (vehicle.track/2) * vehicle.MR_f * 1000;
v_damper_r = omega_rear  * (vehicle.track/2) * vehicle.MR_r * 1000;

% =========================================================================
%% PLOTS
% =========================================================================

% --- Figure 1: Roll Angles ---
figure(1);
plot(t, roll_front_deg, 'LineWidth', 1.5, 'DisplayName', 'Front'); hold on;
plot(t, roll_rear_deg,  'LineWidth', 1.5, 'DisplayName', 'Rear');
yline(roll_ss(1), ':', 'LineWidth', 1.5, 'DisplayName', 'Front SS');
yline(roll_ss(2), ':', 'LineWidth', 1.5, 'DisplayName', 'Rear SS');
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Roll Angles vs Time'); legend; grid on;

% --- Figure 2: Roll Angular Velocities ---
figure(2);
plot(t, x(:,2)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Front'); hold on;
plot(t, x(:,4)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Rear');
xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)');
title('Roll Angular Velocities vs Time'); legend; grid on;

% --- Figure 3: Damper Linear Velocities ---
figure(3);
plot(t, v_damper_f, 'LineWidth', 1.5, 'DisplayName', 'Front'); hold on;
plot(t, v_damper_r, 'LineWidth', 1.5, 'DisplayName', 'Rear');
xlabel('Time (s)'); ylabel('Damper Velocity (mm/s)');
title('Damper Linear Velocity from Roll'); legend; grid on;

% --- Figures 4 & 5: Load Transfer Breakdowns ---
plot_load_transfer(t, ELT_f_spring, ELT_f_damper, GLT_f_vec, TLT_f, 'Front');
plot_load_transfer(t, ELT_r_spring, ELT_r_damper, GLT_r_vec, TLT_r, 'Rear');

% --- Figure 6: Total LLT Verification ---
figure(6);
plot(t, TLT_f,           'LineWidth', 1.5, 'DisplayName', 'Front Total'); hold on;
plot(t, TLT_r,           'LineWidth', 1.5, 'DisplayName', 'Rear Total');
plot(t, TLT_f + TLT_r,   'LineWidth', 1.5, 'DisplayName', 'Combined Total');
plot(t, Total_LLT,        'LineWidth', 1.5, 'DisplayName', 'Vehicle Total');
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Total Lateral Load Transfer Verification'); legend; grid on;


% =========================================================================
%% LOCAL FUNCTIONS
% =========================================================================

function dxdt = eom(~, x, suspension, I, LAT_G, car, front, rear, frontunsprung, rearunsprung)
    [~, M] = roll_angle(LAT_G, car, front, rear, frontunsprung, rearunsprung);
    dxdt = zeros(4, 1);
    dxdt(1) = x(2);
    dxdt(2) = -(suspension.kf/I)*x(1) - (suspension.cf/I)*x(2) + M(1)/I;
    dxdt(3) = x(4);
    dxdt(4) = -(suspension.kr/I)*x(3) - (suspension.cr/I)*x(4) + M(2)/I;
end

function plot_load_transfer(t, ELT_spring, ELT_damper, GLT, TLT, location)
    figure;
    plot(t, ELT_spring, 'LineWidth', 1.5, 'DisplayName', 'Elastic Spring'); hold on;
    plot(t, ELT_damper, 'LineWidth', 1.5, 'DisplayName', 'Elastic Damper');
    plot(t, GLT,        'LineWidth', 1.5, 'DisplayName', 'Geometric');
    plot(t, TLT,        'LineWidth', 1.5, 'DisplayName', 'Total');
    xlabel('Time (s)'); ylabel('Load Transfer (N)');
    title(sprintf('%s Load Transfer Breakdown', location)); legend; grid on;
end

function compute_rise_time(t, signal_deg, ss_val, label)
    if ss_val <= 0
        fprintf('%s: steady-state non-positive, rise time N/A.\n', label);
        return;
    end
    idx_10 = find(signal_deg >= 0.1 * ss_val, 1, 'first');
    idx_90 = find(signal_deg >= 0.9 * ss_val, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        fprintf('%s rise time (10–90%%): %.4f s\n', label, t(idx_90) - t(idx_10));
    else
        fprintf('%s: rise time not detected within time span.\n', label);
    end
end