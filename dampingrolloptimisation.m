%% LATERAL LOAD TRANSFER ANALYSIS - MULTI-TARGET OVERSHOOT OPTIMISATION
clear; clc;
paramR26;  % This loads car, front, rear, etc.

%% -----------------------------------------------------------------------
%  CONSTANTS AND PARAMETERS
%% -----------------------------------------------------------------------
LAT_G         = 1.8;           % Lateral acceleration (g)
ROLL_INERTIA  = 182.24965;     % Roll inertia (kg*m^2)
TIME_SPAN     = [0 1];
INITIAL_STATE = [0 0 0 0];

%% -----------------------------------------------------------------------
%  SUSPENSION PARAMETERS
%% -----------------------------------------------------------------------
suspension       = struct();
suspension.kf    = front.k_roll;
suspension.kr    = rear.k_roll;
suspension.cf    = front.cs_roll;
suspension.cr    = rear.cs_roll;
suspension.RC_f  = front.RC;
suspension.RC_r  = rear.RC;

%% -----------------------------------------------------------------------
%  VEHICLE PARAMETERS
%% -----------------------------------------------------------------------
vehicle          = struct();
vehicle.mass     = car.m;
vehicle.track    = car.track;
vehicle.cgh      = car.cgh;
vehicle.wgt_dist = weight_distribution / 100;

%% -----------------------------------------------------------------------
%  TARGET STEADY-STATE ROLL ANGLES AND MOMENTS
%% -----------------------------------------------------------------------
[R_ss_matrix, M_ss_matrix] = roll_angle(LAT_G, car, front, rear, frontunsprung, rearunsprung);
target_phi_f = R_ss_matrix(1);
target_phi_r = R_ss_matrix(2);
M_ode        = M_ss_matrix(1:2);

%% -----------------------------------------------------------------------
%  OVERSHOOT TARGETS TO OPTIMISE FOR
%% -----------------------------------------------------------------------
overshoot_cases = struct( ...
    'label',            {'Minimal (≈0%)', '5% Overshoot', '10% Overshoot', '15% Overshoot', '20% Overshoot'}, ...
    'target_overshoot', {0.00,             0.05,            0.10,            0.15,            0.20}, ...
    'penalty_weight',   {10.0,             10.0,            10.0,            10.0,            10.0});

num_cases = numel(overshoot_cases);

%% -----------------------------------------------------------------------
%  PRE-ALLOCATE RESULTS STORAGE
%% -----------------------------------------------------------------------
results = struct();
for k = 1:num_cases
    results(k).label          = overshoot_cases(k).label;
    results(k).optimal_cf     = NaN;
    results(k).optimal_cr     = NaN;
    results(k).optimal_cf_lin = NaN;   % Linear damping front (Ns/m)
    results(k).optimal_cr_lin = NaN;   % Linear damping rear  (Ns/m)
    results(k).omega_n_f      = NaN;   % Front natural frequency (rad/s)
    results(k).omega_n_r      = NaN;   % Rear  natural frequency (rad/s)
    results(k).zeta_f         = NaN;   % Front damping ratio (-)
    results(k).zeta_r         = NaN;   % Rear  damping ratio (-)
    results(k).min_cost       = NaN;
    results(k).overshoot_f    = NaN;
    results(k).overshoot_r    = NaN;
    results(k).delay_f        = NaN;
    results(k).delay_r        = NaN;
    results(k).t              = [];
    results(k).x              = [];
    results(k).TLT_f          = [];
    results(k).TLT_r          = [];
    results(k).ELT_f_spring   = [];
    results(k).ELT_f_damper   = [];
    results(k).ELT_r_spring   = [];
    results(k).ELT_r_damper   = [];
    results(k).GLT_f_vec      = [];
    results(k).GLT_r_vec      = [];
end

%% -----------------------------------------------------------------------
%  NESTED FUNCTION: eom
%% -----------------------------------------------------------------------
function dxdt = eom(~, x_ode, current_suspension, roll_inertia, m_ode)
    I    = roll_inertia;
    dxdt = zeros(4, 1);

    dxdt(1) = x_ode(2);
    dxdt(2) = -(current_suspension.kf / I) * x_ode(1) ...
              -(current_suspension.cf / I) * x_ode(2) ...
              + m_ode(1) / I;

    dxdt(3) = x_ode(4);
    dxdt(4) = -(current_suspension.kr / I) * x_ode(3) ...
              -(current_suspension.cr / I) * x_ode(4) ...
              + m_ode(2) / I;
end

%% -----------------------------------------------------------------------
%  NESTED FUNCTION: objective_function
%% -----------------------------------------------------------------------
function [cost, overshoot_vals, delay_vals] = objective_function( ...
        damping_coeffs, ...
        base_suspension, ...
        time_span, ...
        initial_state, ...
        roll_inertia, ...
        m_ode, ...
        tgt_phi_f, ...
        tgt_phi_r, ...
        target_overshoot, ...
        penalty_weight)

    cf_val = damping_coeffs(1);
    cr_val = damping_coeffs(2);

    if cf_val < 0 || cr_val < 0
        cost           = inf;
        overshoot_vals = [inf, inf];
        delay_vals     = [inf, inf];
        return;
    end

    current_suspension    = base_suspension;
    current_suspension.cf = cf_val;
    current_suspension.cr = cr_val;

    opts_ode = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t_opt, x_opt] = ode45( ...
        @(t, x_ode) eom(t, x_ode, current_suspension, roll_inertia, m_ode), ...
        time_span, initial_state, opts_ode);

    % Front metrics
    peak_f = max(x_opt(:, 1));
    if tgt_phi_f ~= 0
        overshoot_f = max(0, (peak_f - tgt_phi_f) / tgt_phi_f);
    else
        overshoot_f = 0;
    end
    idx_f = find(x_opt(:, 1) >= 0.9 * tgt_phi_f, 1, 'first');
    if ~isempty(idx_f); delay_f = t_opt(idx_f); else; delay_f = time_span(2); end

    % Rear metrics
    peak_r = max(x_opt(:, 3));
    if tgt_phi_r ~= 0
        overshoot_r = max(0, (peak_r - tgt_phi_r) / tgt_phi_r);
    else
        overshoot_r = 0;
    end
    idx_r = find(x_opt(:, 3) >= 0.9 * tgt_phi_r, 1, 'first');
    if ~isempty(idx_r); delay_r = t_opt(idx_r); else; delay_r = time_span(2); end

    overshoot_vals = [overshoot_f, overshoot_r];
    delay_vals     = [delay_f,     delay_r];

    if target_overshoot == 0
        cost = mean(overshoot_vals) + 0.05 * sum(delay_vals);
    else
        deviation_f = (overshoot_f - target_overshoot)^2;
        deviation_r = (overshoot_r - target_overshoot)^2;
        cost = penalty_weight * (deviation_f + deviation_r) + 0.05 * sum(delay_vals);
    end
end

%% -----------------------------------------------------------------------
%  OPTIMISATION LOOP
%% -----------------------------------------------------------------------
initial_damping_guess = [suspension.cf, suspension.cr];
options = optimset('Display', 'off', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxIter', 2000);

fprintf('\n========================================================\n');
fprintf('       MULTI-TARGET DAMPING OPTIMISATION\n');
fprintf('========================================================\n');

for k = 1:num_cases
    fprintf('\n[%d/%d] Optimising for: %s ...\n', k, num_cases, overshoot_cases(k).label);

    tgt_os     = overshoot_cases(k).target_overshoot;
    pen_weight = overshoot_cases(k).penalty_weight;

    obj_fn = @(d) objective_function( ...
        d, suspension, TIME_SPAN, INITIAL_STATE, ...
        ROLL_INERTIA, M_ode, target_phi_f, target_phi_r, ...
        tgt_os, pen_weight);

    [opt_d, min_c] = fminsearch(obj_fn, initial_damping_guess, options);

    % ---- Store rotational damping ----
    results(k).optimal_cf = opt_d(1);
    results(k).optimal_cr = opt_d(2);
    results(k).min_cost   = min_c;

    % ---- Linear damping (Ns/m): c_lin = c_rot * 2 / track ----
    track = vehicle.track;
    results(k).optimal_cf_lin = opt_d(1) * 2 / track^2;
    results(k).optimal_cr_lin = opt_d(2) * 2 / track^2;

    % ---- Natural frequencies (rad/s): wn = sqrt(k / I) ----
    omega_n_f = sqrt(suspension.kf / ROLL_INERTIA);
    omega_n_r = sqrt(suspension.kr / ROLL_INERTIA);
    results(k).omega_n_f = omega_n_f;
    results(k).omega_n_r = omega_n_r;

    % ---- Damping ratios: zeta = c / (2 * sqrt(k * I)) ----
    %      Equivalent to: zeta = c / (2 * I * wn)
    c_crit_f = 2 * sqrt(suspension.kf * ROLL_INERTIA);
    c_crit_r = 2 * sqrt(suspension.kr * ROLL_INERTIA);
    results(k).zeta_f = opt_d(1) / c_crit_f;
    results(k).zeta_r = opt_d(2) / c_crit_r;

    % ---- Re-run ODE with optimal damping ----
    susp_opt    = suspension;
    susp_opt.cf = opt_d(1);
    susp_opt.cr = opt_d(2);

    opts_ode = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t_sim, x_sim] = ode45( ...
        @(t, x_ode) eom(t, x_ode, susp_opt, ROLL_INERTIA, M_ode), ...
        TIME_SPAN, INITIAL_STATE, opts_ode);

    [~, final_os, final_dl] = objective_function( ...
        opt_d, suspension, TIME_SPAN, INITIAL_STATE, ...
        ROLL_INERTIA, M_ode, target_phi_f, target_phi_r, ...
        tgt_os, pen_weight);

    results(k).overshoot_f = final_os(1);
    results(k).overshoot_r = final_os(2);
    results(k).delay_f     = final_dl(1);
    results(k).delay_r     = final_dl(2);
    results(k).t           = t_sim;
    results(k).x           = x_sim;

    % ---- Load Transfer ----
    GLT_f = (vehicle.mass * vehicle.wgt_dist     * 9.81 * LAT_G * susp_opt.RC_f) / vehicle.track;
    GLT_r = (vehicle.mass * (1-vehicle.wgt_dist) * 9.81 * LAT_G * susp_opt.RC_r) / vehicle.track;

    results(k).ELT_f_spring = x_sim(:,1) * susp_opt.kf;
    results(k).ELT_r_spring = x_sim(:,3) * susp_opt.kr;
    results(k).ELT_f_damper = x_sim(:,2) * susp_opt.cf;
    results(k).ELT_r_damper = x_sim(:,4) * susp_opt.cr;

    n_pts = length(t_sim);
    results(k).GLT_f_vec = GLT_f * ones(n_pts, 1);
    results(k).GLT_r_vec = GLT_r * ones(n_pts, 1);

    results(k).TLT_f = results(k).ELT_f_spring + results(k).ELT_f_damper + results(k).GLT_f_vec;
    results(k).TLT_r = results(k).ELT_r_spring + results(k).ELT_r_damper + results(k).GLT_r_vec;
end

%% -----------------------------------------------------------------------
%  PRINT RESULTS TABLE
%% -----------------------------------------------------------------------
fprintf('\n');
fprintf('==========================================================================================================================================\n');
fprintf('                                                    OPTIMISATION SUMMARY\n');
fprintf('==========================================================================================================================================\n');
fprintf('%-18s | %14s | %14s | %13s | %13s | %10s | %10s | %9s | %9s | %10s | %10s | %10s | %10s\n', ...
    'Case', ...
    'cf (N-m-s/rad)', 'cr (N-m-s/rad)', ...
    'cf_lin (Ns/m)',  'cr_lin (Ns/m)', ...
    'wn_f (r/s)',     'wn_r (rad/s)', ...
    'zeta_f',         'zeta_r', ...
    'OS_f (%)',        'OS_r (%)', ...
    'Rise_f (s)',      'Rise_r (s)');
fprintf('%s\n', repmat('-', 1, 158));

for k = 1:num_cases
    fprintf('%-18s | %14.4f | %14.4f | %13.4f | %13.4f | %10.4f | %10.4f | %9.4f | %9.4f | %10.2f | %10.2f | %10.4f | %10.4f\n', ...
        results(k).label, ...
        results(k).optimal_cf,     results(k).optimal_cr, ...
        results(k).optimal_cf_lin, results(k).optimal_cr_lin, ...
        results(k).omega_n_f,      results(k).omega_n_r, ...
        results(k).zeta_f,         results(k).zeta_r, ...
        results(k).overshoot_f * 100, results(k).overshoot_r * 100, ...
        results(k).delay_f,        results(k).delay_r);
end
fprintf('==========================================================================================================================================\n');

%% -----------------------------------------------------------------------
%  FIGURE 1: Front Roll Angle Comparison
%% -----------------------------------------------------------------------
figure(1); clf; hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).x(:,1)*180/pi, 'LineWidth', 2, 'DisplayName', results(k).label);
end
yline(target_phi_f*180/pi, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Front SS Target (%.3f deg)', target_phi_f*180/pi));
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Front Roll Angle — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURE 2: Rear Roll Angle Comparison
%% -----------------------------------------------------------------------
figure(2); clf; hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).x(:,3)*180/pi, 'LineWidth', 2, 'DisplayName', results(k).label);
end
yline(target_phi_r*180/pi, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Rear SS Target (%.3f deg)', target_phi_r*180/pi));
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Rear Roll Angle — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURE 3: Front Angular Velocity Comparison
%% -----------------------------------------------------------------------
figure(3); clf; hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).x(:,2)*180/pi, 'LineWidth', 2, 'DisplayName', results(k).label);
end
xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)');
title('Front Roll Angular Velocity — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURE 4: Rear Angular Velocity Comparison
%% -----------------------------------------------------------------------
figure(4); clf; hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).x(:,4)*180/pi, 'LineWidth', 2, 'DisplayName', results(k).label);
end
xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)');
title('Rear Roll Angular Velocity — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURE 5: Front Total Load Transfer Comparison
%% -----------------------------------------------------------------------
figure(5); clf; hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).TLT_f, 'LineWidth', 2, 'DisplayName', results(k).label);
end
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Front Total Lateral Load Transfer — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURE 6: Rear Total Load Transfer Comparison
%% -----------------------------------------------------------------------
figure(6); clf; hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).TLT_r, 'LineWidth', 2, 'DisplayName', results(k).label);
end
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Rear Total Lateral Load Transfer — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURES 7-16: Per-Case Load Transfer Breakdown (Front + Rear)
%% -----------------------------------------------------------------------
for k = 1:num_cases
    figure(7 + (k-1)*2 + 1); clf;
    plot(results(k).t, results(k).ELT_f_spring, 'LineWidth', 1.5, 'DisplayName', 'ELT Spring');
    hold on;
    plot(results(k).t, results(k).ELT_f_damper, 'LineWidth', 1.5, 'DisplayName', 'ELT Damper');
    plot(results(k).t, results(k).GLT_f_vec,    'LineWidth', 1.5, 'DisplayName', 'GLT');
    plot(results(k).t, results(k).TLT_f,        'LineWidth', 2.0, 'DisplayName', 'TLT Total');
    xlabel('Time (s)'); ylabel('Load Transfer (N)');
    title(sprintf('Front Load Transfer Breakdown — %s', results(k).label));
    legend('Location', 'best'); grid on; hold off;

    figure(7 + (k-1)*2 + 2); clf;
    plot(results(k).t, results(k).ELT_r_spring, 'LineWidth', 1.5, 'DisplayName', 'ELT Spring');
    hold on;
    plot(results(k).t, results(k).ELT_r_damper, 'LineWidth', 1.5, 'DisplayName', 'ELT Damper');
    plot(results(k).t, results(k).GLT_r_vec,    'LineWidth', 1.5, 'DisplayName', 'GLT');
    plot(results(k).t, results(k).TLT_r,        'LineWidth', 2.0, 'DisplayName', 'TLT Total');
    xlabel('Time (s)'); ylabel('Load Transfer (N)');
    title(sprintf('Rear Load Transfer Breakdown — %s', results(k).label));
    legend('Location', 'best'); grid on; hold off;
end

%% -----------------------------------------------------------------------
%  FIGURE 17: Bar Chart — Optimal Damping Coefficients (Rotational)
%% -----------------------------------------------------------------------
figure(17); clf;
case_labels = {results.label};
cf_vals     = [results.optimal_cf];
cr_vals     = [results.optimal_cr];
x_pos       = 1:num_cases;
bar_width   = 0.35;

bar(x_pos - bar_width/2, cf_vals, bar_width, 'FaceColor', [0.2 0.5 0.8], 'DisplayName', 'Front cf');
hold on;
bar(x_pos + bar_width/2, cr_vals, bar_width, 'FaceColor', [0.8 0.3 0.2], 'DisplayName', 'Rear cr');
set(gca, 'XTick', x_pos, 'XTickLabel', case_labels);
ylabel('Damping Coefficient (N-m-s/rad)');
title('Optimal Damping Coefficients — All Cases (Rotational)');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURE 18: Bar Chart — Optimal Damping Coefficients (Linear)
%% -----------------------------------------------------------------------
figure(18); clf;
cf_lin_vals = [results.optimal_cf_lin];
cr_lin_vals = [results.optimal_cr_lin];

bar(x_pos - bar_width/2, cf_lin_vals, bar_width, 'FaceColor', [0.2 0.5 0.8], 'DisplayName', 'Front cf_lin');
hold on;
bar(x_pos + bar_width/2, cr_lin_vals, bar_width, 'FaceColor', [0.8 0.3 0.2], 'DisplayName', 'Rear cr_lin');
set(gca, 'XTick', x_pos, 'XTickLabel', case_labels);
ylabel('Damping Coefficient (Ns/m)');
title('Optimal Damping Coefficients — All Cases (Linear, Ns/m)');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURE 19: Bar Chart — Damping Ratio (zeta) and Natural Frequency
%% -----------------------------------------------------------------------
figure(19); clf;

% --- Subplot 1: Damping Ratio zeta ---
subplot(2, 1, 1);
zeta_f_vals = [results.zeta_f];
zeta_r_vals = [results.zeta_r];
bar(x_pos - bar_width/2, zeta_f_vals, bar_width, 'FaceColor', [0.3 0.7 0.5], 'DisplayName', 'Front zeta');
hold on;
bar(x_pos + bar_width/2, zeta_r_vals, bar_width, 'FaceColor', [0.8 0.5 0.1], 'DisplayName', 'Rear zeta');
yline(1.0, 'k--', 'Critical (zeta=1)', 'LineWidth', 1.5);
yline(0.7, 'b:',  'zeta=0.7',          'LineWidth', 1.2);
set(gca, 'XTick', x_pos, 'XTickLabel', case_labels);
ylabel('Damping Ratio zeta (-)');
title('Damping Ratio — All Cases');
legend('Location', 'best'); grid on; hold off;
sgtitle('System Dynamics: Damping Ratio — All Overshoot Cases');

%% -----------------------------------------------------------------------
%  FIGURE 20: Achieved Overshoot vs Target
%% -----------------------------------------------------------------------
figure(20); clf;
os_f_vals = [results.overshoot_f] * 100;
os_r_vals = [results.overshoot_r] * 100;

bar(x_pos - bar_width/2, os_f_vals, bar_width, 'FaceColor', [0.2 0.7 0.4], 'DisplayName', 'Front OS%');
hold on;
bar(x_pos + bar_width/2, os_r_vals, bar_width, 'FaceColor', [0.9 0.6 0.1], 'DisplayName', 'Rear OS%');
yline(5,  'b--', '5% Target',  'LineWidth', 1.2);
yline(10, 'r--', '10% Target', 'LineWidth', 1.2);
yline(15, 'm--', '15% Target', 'LineWidth', 1.2);
yline(20, 'g--', '20% Target', 'LineWidth', 1.2);
set(gca, 'XTick', x_pos, 'XTickLabel', case_labels);
ylabel('Overshoot (%)');
title('Achieved Overshoot vs Target — All Cases');
legend('Location', 'best'); grid on; hold off;

%% -----------------------------------------------------------------------
%  FIGURE 21: Sensitivity Study — Overshoot vs Rise Time
%% -----------------------------------------------------------------------
overshoot_f_values = [results.overshoot_f] * 100;
overshoot_r_values = [results.overshoot_r] * 100;
rise_time_f_values = [results.delay_f];
rise_time_r_values = [results.delay_r];

figure(21); clf;
subplot(2, 1, 1); hold on;
plot(rise_time_f_values, overshoot_f_values, '-o', 'LineWidth', 2, 'DisplayName', 'Front Overshoot');
xlabel('Rise Time (s)'); ylabel('Overshoot (%)');
title('Front Overshoot vs. Rise Time');
grid on; legend('Location', 'best');

subplot(2, 1, 2); hold on;
plot(rise_time_r_values, overshoot_r_values, '-o', 'LineWidth', 2, 'DisplayName', 'Rear Overshoot');
xlabel('Rise Time (s)'); ylabel('Overshoot (%)');
title('Rear Overshoot vs. Rise Time');
grid on; legend('Location', 'best');

sgtitle('Sensitivity Study: Overshoot Percentage vs. Rise Time');
