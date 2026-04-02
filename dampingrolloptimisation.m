%% LATERAL LOAD TRANSFER ANALYSIS - OPTIMISED DAMPING VERSION
clear; clc;
paramR26;  % This loads car, front, rear, etc.

%% Constants and Parameters
LAT_G        = 1;           % Lateral acceleration (g)
ROLL_INERTIA = 182.24965;   % Roll inertia (kg*m^2)
TIME_SPAN    = [0 1];
INITIAL_STATE = [0 0 0 0];

%% Extract and Pre-compute Suspension Parameters
suspension        = struct();
suspension.kf     = front.k_roll;
suspension.kr     = rear.k_roll;
suspension.cf     = front.cs_roll;   % Initial guess for front damping
suspension.cr     = rear.cs_roll;    % Initial guess for rear damping
suspension.RC_f   = front.RC;
suspension.RC_r   = rear.RC;

% Vehicle parameters
vehicle           = struct();
vehicle.mass      = car.m;
vehicle.track     = car.track;
vehicle.cgh       = car.cgh;
vehicle.wgt_dist  = weight_distribution / 100;

%% Calculate Target Steady-State Roll Angles and Moments
[R_ss_matrix, M_ss_matrix] = roll_angle(LAT_G, car, front, rear, frontunsprung, rearunsprung);
target_phi_f = R_ss_matrix(1);   % Steady-state front roll angle (rad)
target_phi_r = R_ss_matrix(2);   % Steady-state rear roll angle  (rad)
M_ode        = M_ss_matrix(1:2); % Roll moments [front; rear]

%% -----------------------------------------------------------------------
%  NESTED FUNCTION: eom
%  All required constants are passed explicitly to avoid scope errors.
%% -----------------------------------------------------------------------
function dxdt = eom(~, x_ode, current_suspension, roll_inertia, m_ode)
    I     = roll_inertia;
    dxdt  = zeros(4, 1);

    % Front roll dynamics
    dxdt(1) = x_ode(2);
    dxdt(2) = -(current_suspension.kf / I) * x_ode(1) ...
              -(current_suspension.cf / I) * x_ode(2) ...
              + m_ode(1) / I;

    % Rear roll dynamics
    dxdt(3) = x_ode(4);
    dxdt(4) = -(current_suspension.kr / I) * x_ode(3) ...
              -(current_suspension.cr / I) * x_ode(4) ...
              + m_ode(2) / I;
end

%% -----------------------------------------------------------------------
%  NESTED FUNCTION: objective_function
%  Every variable it needs is passed in — no implicit scoping.
%% -----------------------------------------------------------------------
function [cost, overshoot_vals, delay_vals] = objective_function( ...
        damping_coeffs_opt, ...
        base_suspension, ...
        time_span, ...
        initial_state, ...
        roll_inertia, ...
        m_ode, ...
        tgt_phi_f, ...
        tgt_phi_r)

    cf_val = damping_coeffs_opt(1);
    cr_val = damping_coeffs_opt(2);

    % Penalise non-physical negative damping
    if cf_val < 0 || cr_val < 0
        cost          = inf;
        overshoot_vals = [inf, inf];
        delay_vals     = [inf, inf];
        return;
    end

    % Build current suspension struct for this iteration
    current_suspension    = base_suspension;
    current_suspension.cf = cf_val;
    current_suspension.cr = cr_val;

    % Solve ODE — pass roll_inertia and m_ode explicitly into eom
    [t_opt, x_opt] = ode45( ...
        @(t, x_ode) eom(t, x_ode, current_suspension, roll_inertia, m_ode), ...
        time_span, initial_state);

    % ----- Front Axle Metrics -----
    peak_f = max(x_opt(:, 1));
    if tgt_phi_f ~= 0
        overshoot_f = max(0, (peak_f - tgt_phi_f) / tgt_phi_f);
    else
        overshoot_f = 0;
    end

    idx_f = find(x_opt(:, 1) >= 0.9 * tgt_phi_f, 1, 'first');
    if ~isempty(idx_f)
        delay_f = t_opt(idx_f);
    else
        delay_f = time_span(2);   % 90% never reached in time window
    end

    % ----- Rear Axle Metrics -----
    peak_r = max(x_opt(:, 3));
    if tgt_phi_r ~= 0
        overshoot_r = max(0, (peak_r - tgt_phi_r) / tgt_phi_r);
    else
        overshoot_r = 0;
    end

    idx_r = find(x_opt(:, 3) >= 0.9 * tgt_phi_r, 1, 'first');
    if ~isempty(idx_r)
        delay_r = t_opt(idx_r);
    else
        delay_r = time_span(2);   % 90% never reached in time window
    end

    overshoot_vals = [overshoot_f, overshoot_r];
    delay_vals     = [delay_f,     delay_r];

    % Minimise overshoot first; small delay penalty to break ties
    cost = sum(overshoot_vals) + 0.1 * sum(delay_vals);
end

%% -----------------------------------------------------------------------
%  OPTIMISATION
%  Every variable is captured inside the anonymous function handle.
%% -----------------------------------------------------------------------
initial_damping_guess = [suspension.cf, suspension.cr];

options = optimset('Display', 'iter', 'TolX', 1e-4, 'TolFun', 1e-4);

[optimal_damping, ~] = fminsearch( ...
    @(d) objective_function(d, suspension, TIME_SPAN, INITIAL_STATE, ...
                             ROLL_INERTIA, M_ode, target_phi_f, target_phi_r), ...
    initial_damping_guess, options);

optimal_cf = optimal_damping(1);
optimal_cr = optimal_damping(2);

%% -----------------------------------------------------------------------
%  POST-OPTIMISATION: Re-run with optimal damping
%% -----------------------------------------------------------------------
suspension.cf = optimal_cf;
suspension.cr = optimal_cr;

[t, x] = ode45( ...
    @(t, x_ode) eom(t, x_ode, suspension, ROLL_INERTIA, M_ode), ...
    TIME_SPAN, INITIAL_STATE);

% Retrieve final metrics
[~, final_overshoot, final_delay] = objective_function( ...
    optimal_damping, suspension, TIME_SPAN, INITIAL_STATE, ...
    ROLL_INERTIA, M_ode, target_phi_f, target_phi_r);

fprintf('\n=== Optimisation Results ===\n');
fprintf('Optimal Front Damping (cf) : %.4f N-m-s/rad\n', optimal_cf);
fprintf('Optimal Rear  Damping (cr) : %.4f N-m-s/rad\n', optimal_cr);
fprintf('\n--- Response Metrics (Optimal Damping) ---\n');
fprintf('Front Roll Overshoot       : %.2f%%\n',   final_overshoot(1) * 100);
fprintf('Front Roll 90%% Rise Time  : %.4f s\n',   final_delay(1));
fprintf('Rear  Roll Overshoot       : %.2f%%\n',   final_overshoot(2) * 100);
fprintf('Rear  Roll 90%% Rise Time  : %.4f s\n',   final_delay(2));

%% -----------------------------------------------------------------------
%  LOAD TRANSFER CALCULATIONS (vectorised)
%% -----------------------------------------------------------------------
GLT_f = (vehicle.mass * vehicle.wgt_dist       * 9.81 * LAT_G * suspension.RC_f) / vehicle.track;
GLT_r = (vehicle.mass * (1-vehicle.wgt_dist)   * 9.81 * LAT_G * suspension.RC_r) / vehicle.track;

ELT_f_spring = x(:,1) * suspension.kf;
ELT_r_spring = x(:,3) * suspension.kr;
ELT_f_damper = x(:,2) * suspension.cf;
ELT_r_damper = x(:,4) * suspension.cr;

n_pts      = length(t);
GLT_f_vec  = GLT_f * ones(n_pts, 1);
GLT_r_vec  = GLT_r * ones(n_pts, 1);

TLT_f      = ELT_f_spring + ELT_f_damper + GLT_f_vec;
TLT_r      = ELT_r_spring + ELT_r_damper + GLT_r_vec;
Total_LLT  = (LAT_G * 9.81 * vehicle.mass * vehicle.cgh / vehicle.track) * ones(n_pts, 1);

%% -----------------------------------------------------------------------
%  NESTED FUNCTION: plot_load_transfer
%% -----------------------------------------------------------------------
function plot_load_transfer(t_vals, elt_spring, elt_damper, glt, tlt, location)
    figure;
    plot(t_vals, elt_spring, 'LineWidth', 1.5, 'DisplayName', 'Elastic Spring');
    hold on;
    plot(t_vals, elt_damper, 'LineWidth', 1.5, 'DisplayName', 'Elastic Damper');
    plot(t_vals, glt,        'LineWidth', 1.5, 'DisplayName', 'Geometric');
    plot(t_vals, tlt,        'LineWidth', 1.5, 'DisplayName', 'Total');
    xlabel('Time (s)');
    ylabel('Load Transfer (N)');
    title(sprintf('%s Load Transfer Breakdown (Optimal Damping)', location));
    legend('Location', 'best');
    grid on;
end

%% -----------------------------------------------------------------------
%  PLOTS
%% -----------------------------------------------------------------------
% Figure 1: Roll Angles
figure(1); clf;
plot(t, x(:,1)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Front Roll Angle');
hold on;
plot(t, x(:,3)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Rear Roll Angle');
yline(target_phi_f*180/pi, '--r', 'LineWidth', 1.2, ...
    'DisplayName', sprintf('Front SS Target (%.3f deg)', target_phi_f*180/pi));
yline(target_phi_r*180/pi, '--b', 'LineWidth', 1.2, ...
    'DisplayName', sprintf('Rear SS Target (%.3f deg)',  target_phi_r*180/pi));
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Roll Angles vs Time (Optimal Damping)');
legend('Location', 'best'); grid on; hold off;

% Figure 2: Angular Velocities
figure(2); clf;
plot(t, x(:,2)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Front ω');
hold on;
plot(t, x(:,4)*180/pi, 'LineWidth', 1.5, 'DisplayName', 'Rear ω');
xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)');
title('Roll Angular Velocities vs Time (Optimal Damping)');
legend('Location', 'best'); grid on; hold off;

% Figures 3 & 4: Load Transfer Breakdowns
plot_load_transfer(t, ELT_f_spring, ELT_f_damper, GLT_f_vec, TLT_f, 'Front');
plot_load_transfer(t, ELT_r_spring, ELT_r_damper, GLT_r_vec, TLT_r, 'Rear');

% Figure 5: Total LLT Verification
figure(5); clf;
plot(t, TLT_f,         'LineWidth', 1.5, 'DisplayName', 'Front Total');
hold on;
plot(t, TLT_r,         'LineWidth', 1.5, 'DisplayName', 'Rear Total');
plot(t, TLT_f + TLT_r, 'LineWidth', 1.5, 'DisplayName', 'Combined Total');
plot(t, Total_LLT,     'LineWidth', 1.5, 'DisplayName', 'Vehicle Total (Check)');
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Total Lateral Load Transfer Verification (Optimal Damping)');
legend('Location', 'best'); grid on; hold off;
