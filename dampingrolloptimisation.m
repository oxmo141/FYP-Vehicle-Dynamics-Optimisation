%% LATERAL LOAD TRANSFER ANALYSIS - MULTI-TARGET OVERSHOOT OPTIMISATION
clear; clc;
paramR26;  % This loads car, front, rear, etc.

%% -----------------------------------------------------------------------
%  CONSTANTS AND PARAMETERS
%% -----------------------------------------------------------------------
LAT_G         = 1.3;           % Lateral acceleration (g)
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
    results(k).optimal_cf_lin = NaN;
    results(k).optimal_cr_lin = NaN;
    results(k).omega_n_f      = NaN;
    results(k).omega_n_r      = NaN;
    results(k).zeta_f         = NaN;
    results(k).zeta_r         = NaN;
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

    peak_f = max(x_opt(:, 1));
    if tgt_phi_f ~= 0
        overshoot_f = max(0, (peak_f - tgt_phi_f) / tgt_phi_f);
    else
        overshoot_f = 0;
    end
    idx_f = find(x_opt(:, 1) >= 0.9 * tgt_phi_f, 1, 'first');
    if ~isempty(idx_f); delay_f = t_opt(idx_f); else; delay_f = time_span(2); end

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

    results(k).optimal_cf = opt_d(1);
    results(k).optimal_cr = opt_d(2);
    results(k).min_cost   = min_c;

    track = vehicle.track;
    results(k).optimal_cf_lin = opt_d(1) * 2 / track^2;
    results(k).optimal_cr_lin = opt_d(2) * 2 / track^2;

    omega_n_f = sqrt(suspension.kf / ROLL_INERTIA);
    omega_n_r = sqrt(suspension.kr / ROLL_INERTIA);
    results(k).omega_n_f = omega_n_f;
    results(k).omega_n_r = omega_n_r;

    c_crit_f = 2 * sqrt(suspension.kf * ROLL_INERTIA);
    c_crit_r = 2 * sqrt(suspension.kr * ROLL_INERTIA);
    results(k).zeta_f = opt_d(1) / c_crit_f;
    results(k).zeta_r = opt_d(2) / c_crit_r;

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
%  HELPER: apply black theme to current axes
%% -----------------------------------------------------------------------
function apply_dark(ax)
    set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', ...
            'GridColor', 'w', 'GridAlpha', 0.15, 'MinorGridColor', 'w');
    ax.Title.Color  = 'w';
    ax.XLabel.Color = 'w';
    ax.YLabel.Color = 'w';
    lg = ax.Legend;
    if ~isempty(lg)
        set(lg, 'Color', [0.1 0.1 0.1], 'TextColor', 'w', 'EdgeColor', 'w');
    end
end

function fig = dark_figure(num)
    fig = figure(num); clf;
    set(fig, 'Color', 'k', 'InvertHardcopy', 'off');
end

%% -----------------------------------------------------------------------
%  FIGURE 1: Front Roll Angle Comparison
%% -----------------------------------------------------------------------
dark_figure(1); hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).x(:,1)*180/pi, 'LineWidth', 2, 'DisplayName', results(k).label);
end
yl = yline(target_phi_f*180/pi, '--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Front SS Target (%.3f deg)', target_phi_f*180/pi));
yl.Color = 'w';
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Front Roll Angle — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURE 2: Rear Roll Angle Comparison
%% -----------------------------------------------------------------------
dark_figure(2); hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).x(:,3)*180/pi, 'LineWidth', 2, 'DisplayName', results(k).label);
end
yl = yline(target_phi_r*180/pi, '--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Rear SS Target (%.3f deg)', target_phi_r*180/pi));
yl.Color = 'w';
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Rear Roll Angle — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURE 3: Front Angular Velocity Comparison
%% -----------------------------------------------------------------------
dark_figure(3); hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).x(:,2)*180/pi, 'LineWidth', 2, 'DisplayName', results(k).label);
end
xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)');
title('Front Roll Angular Velocity — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURE 4: Rear Angular Velocity Comparison
%% -----------------------------------------------------------------------
dark_figure(4); hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).x(:,4)*180/pi, 'LineWidth', 2, 'DisplayName', results(k).label);
end
xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)');
title('Rear Roll Angular Velocity — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURE 5: Front Total Load Transfer Comparison
%% -----------------------------------------------------------------------
dark_figure(5); hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).TLT_f, 'LineWidth', 2, 'DisplayName', results(k).label);
end
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Front Total Lateral Load Transfer — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURE 6: Rear Total Load Transfer Comparison
%% -----------------------------------------------------------------------
dark_figure(6); hold on;
for k = 1:num_cases
    plot(results(k).t, results(k).TLT_r, 'LineWidth', 2, 'DisplayName', results(k).label);
end
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Rear Total Lateral Load Transfer — All Overshoot Cases');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURES 7-16: Per-Case Load Transfer Breakdown (Front + Rear)
%% -----------------------------------------------------------------------
for k = 1:num_cases
    dark_figure(7 + (k-1)*2 + 1);
    plot(results(k).t, results(k).ELT_f_spring, 'LineWidth', 1.5, 'DisplayName', 'ELT Spring');
    hold on;
    plot(results(k).t, results(k).ELT_f_damper, 'LineWidth', 1.5, 'DisplayName', 'ELT Damper');
    plot(results(k).t, results(k).GLT_f_vec,    'LineWidth', 1.5, 'DisplayName', 'GLT');
    plot(results(k).t, results(k).TLT_f,        'LineWidth', 2.0, 'DisplayName', 'TLT Total');
    xlabel('Time (s)'); ylabel('Load Transfer (N)');
    title(sprintf('Front Load Transfer Breakdown — %s', results(k).label));
    legend('Location', 'best'); grid on; hold off;
    apply_dark(gca);

    dark_figure(7 + (k-1)*2 + 2);
    plot(results(k).t, results(k).ELT_r_spring, 'LineWidth', 1.5, 'DisplayName', 'ELT Spring');
    hold on;
    plot(results(k).t, results(k).ELT_r_damper, 'LineWidth', 1.5, 'DisplayName', 'ELT Damper');
    plot(results(k).t, results(k).GLT_r_vec,    'LineWidth', 1.5, 'DisplayName', 'GLT');
    plot(results(k).t, results(k).TLT_r,        'LineWidth', 2.0, 'DisplayName', 'TLT Total');
    xlabel('Time (s)'); ylabel('Load Transfer (N)');
    title(sprintf('Rear Load Transfer Breakdown — %s', results(k).label));
    legend('Location', 'best'); grid on; hold off;
    apply_dark(gca);
end

%% -----------------------------------------------------------------------
%  FIGURE 17: Bar Chart — Optimal Damping Coefficients (Rotational)
%% -----------------------------------------------------------------------
dark_figure(17);
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
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURE 18: Bar Chart — Optimal Damping Coefficients (Linear)
%% -----------------------------------------------------------------------
dark_figure(18);
cf_lin_vals = [results.optimal_cf_lin];
cr_lin_vals = [results.optimal_cr_lin];

bar(x_pos - bar_width/2, cf_lin_vals, bar_width, 'FaceColor', [0.2 0.5 0.8], 'DisplayName', 'Front cf');
hold on;
bar(x_pos + bar_width/2, cr_lin_vals, bar_width, 'FaceColor', [0.8 0.3 0.2], 'DisplayName', 'Rear cr');
set(gca, 'XTick', x_pos, 'XTickLabel', case_labels);
ylabel('Damping Coefficient (Ns/m)');
title('Optimal Damping Coefficients — All Cases (Linear, Ns/m)');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURE 19: Bar Chart — Damping Ratio (zeta) and Natural Frequency
%% -----------------------------------------------------------------------
fig19 = dark_figure(19);

subplot(2, 1, 1);
zeta_f_vals = [results.zeta_f];
zeta_r_vals = [results.zeta_r];
bar(x_pos - bar_width/2, zeta_f_vals, bar_width, 'FaceColor', [0.3 0.7 0.5], 'DisplayName', 'Front zeta');
hold on;
bar(x_pos + bar_width/2, zeta_r_vals, bar_width, 'FaceColor', [0.8 0.5 0.1], 'DisplayName', 'Rear zeta');
yl1 = yline(1.0, '--', 'Critical (zeta=1)', 'LineWidth', 1.5); yl1.Color = 'w';
yl2 = yline(0.7, ':',  'zeta=0.7',          'LineWidth', 1.2); yl2.Color = 'w';
set(gca, 'XTick', x_pos, 'XTickLabel', case_labels);
ylabel('Damping Ratio zeta (-)');
title('Damping Ratio — All Cases');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

sg = sgtitle('System Dynamics: Damping Ratio — All Overshoot Cases');
sg.Color = 'w';

%% -----------------------------------------------------------------------
%  FIGURE 20: Achieved Overshoot vs Target
%% -----------------------------------------------------------------------
dark_figure(20);
os_f_vals = [results.overshoot_f] * 100;
os_r_vals = [results.overshoot_r] * 100;

bar(x_pos - bar_width/2, os_f_vals, bar_width, 'FaceColor', [0.2 0.7 0.4], 'DisplayName', 'Front OS%');
hold on;
bar(x_pos + bar_width/2, os_r_vals, bar_width, 'FaceColor', [0.9 0.6 0.1], 'DisplayName', 'Rear OS%');
yl5  = yline(5,  '--', '5% Target',  'LineWidth', 1.2); yl5.Color  = [0.4 0.6 1.0];
yl10 = yline(10, '--', '10% Target', 'LineWidth', 1.2); yl10.Color = [1.0 0.4 0.4];
yl15 = yline(15, '--', '15% Target', 'LineWidth', 1.2); yl15.Color = [1.0 0.4 1.0];
yl20 = yline(20, '--', '20% Target', 'LineWidth', 1.2); yl20.Color = [0.4 1.0 0.4];
set(gca, 'XTick', x_pos, 'XTickLabel', case_labels);
ylabel('Overshoot (%)');
title('Achieved Overshoot vs Target — All Cases');
legend('Location', 'best'); grid on; hold off;
apply_dark(gca);

%% -----------------------------------------------------------------------
%  FIGURE 21: Sensitivity Study — Overshoot vs Rise Time + Summary Table
%% -----------------------------------------------------------------------
overshoot_f_values = [results.overshoot_f] * 100;
overshoot_r_values = [results.overshoot_r] * 100;
rise_time_f_values = [results.delay_f];
rise_time_r_values = [results.delay_r];

% --- Compute step-by-step % increase in rise time ---
n = num_cases;
rise_f = rise_time_f_values(:);
rise_r = rise_time_r_values(:);
os_f   = overshoot_f_values(:);
os_r   = overshoot_r_values(:);

delta_rise_f = zeros(n, 1);
delta_rise_r = zeros(n, 1);
for i = 2:n
    delta_rise_f(i) = 100 * (rise_f(i) - rise_f(i-1)) / rise_f(i-1);
    delta_rise_r(i) = 100 * (rise_r(i) - rise_r(i-1)) / rise_r(i-1);
end

cum_rise_f = zeros(n, 1);
cum_rise_r = zeros(n, 1);
for i = 2:n
    cum_rise_f(i) = 100 * (rise_f(i) - rise_f(1)) / rise_f(1);
    cum_rise_r(i) = 100 * (rise_r(i) - rise_r(1)) / rise_r(1);
end

% --- Print to command window ---
fprintf('\n');
fprintf('==================================================================================================\n');
fprintf('         RISE TIME SENSITIVITY TABLE — %% Increase per 5%% Overshoot Step\n');
fprintf('==================================================================================================\n');
fprintf('%-18s | %10s | %10s | %14s | %14s | %14s | %14s\n', ...
    'Case', 'OS_f (%)', 'OS_r (%)', 'Rise_f (s)', 'Rise_r (s)', 'D Rise_f (%)', 'D Rise_r (%)');
fprintf('%s\n', repmat('-', 1, 100));
for i = 1:n
    if i == 1; df_str = 'baseline'; dr_str = 'baseline';
    else
        df_str = sprintf('%+.2f%%', delta_rise_f(i));
        dr_str = sprintf('%+.2f%%', delta_rise_r(i));
    end
    fprintf('%-18s | %10.2f | %10.2f | %14.4f | %14.4f | %14s | %14s\n', ...
        results(i).label, os_f(i), os_r(i), rise_f(i), rise_r(i), df_str, dr_str);
end
fprintf('%s\n', repmat('-', 1, 100));
fprintf('%-18s | %10s | %10s | %14s | %14s | %14s | %14s\n', ...
    'TOTAL (vs base)', '', '', ...
    sprintf('%.4f', rise_f(end)), sprintf('%.4f', rise_r(end)), ...
    sprintf('%+.2f%%', cum_rise_f(end)), sprintf('%+.2f%%', cum_rise_r(end)));
fprintf('==================================================================================================\n');

% --- Build table string data ---
col_headers = {'Case', 'OS Front (%)', 'OS Rear (%)', ...
               'Rise Front (s)', 'Rise Rear (s)', ...
               'Delta Rise Front', 'Delta Rise Rear'};
table_str = cell(n, 7);
for i = 1:n
    if i == 1; df_str = 'baseline'; dr_str = 'baseline';
    else
        df_str = sprintf('%+.2f%%', delta_rise_f(i));
        dr_str = sprintf('%+.2f%%', delta_rise_r(i));
    end
    table_str{i,1} = results(i).label;
    table_str{i,2} = sprintf('%.2f', os_f(i));
    table_str{i,3} = sprintf('%.2f', os_r(i));
    table_str{i,4} = sprintf('%.4f', rise_f(i));
    table_str{i,5} = sprintf('%.4f', rise_r(i));
    table_str{i,6} = df_str;
    table_str{i,7} = dr_str;
end

% --- Figure 21: 3-panel layout (top-left, top-right, bottom table) ---
fig21 = dark_figure(21);
fig21.Position = [100, 100, 1300, 820];

% Top-left: Front plot
ax_f = subplot('Position', [0.06, 0.42, 0.42, 0.50]);
hold(ax_f, 'on');
plot(ax_f, rise_time_f_values, overshoot_f_values, '-o', ...
    'LineWidth', 2, 'Color', 'c', 'DisplayName', 'Front Overshoot');
xlabel(ax_f, 'Rise Time (s)'); ylabel(ax_f, 'Overshoot (%)');
title(ax_f, 'Front Overshoot vs. Rise Time');
grid(ax_f, 'on'); legend(ax_f, 'Location', 'best'); hold(ax_f, 'off');
apply_dark(ax_f);

% Top-right: Rear plot
ax_r = subplot('Position', [0.55, 0.42, 0.42, 0.50]);
hold(ax_r, 'on');
plot(ax_r, rise_time_r_values, overshoot_r_values, '-o', ...
    'LineWidth', 2, 'Color', 'm', 'DisplayName', 'Rear Overshoot');
xlabel(ax_r, 'Rise Time (s)'); ylabel(ax_r, 'Overshoot (%)');
title(ax_r, 'Rear Overshoot vs. Rise Time');
grid(ax_r, 'on'); legend(ax_r, 'Location', 'best'); hold(ax_r, 'off');
apply_dark(ax_r);

sg21 = sgtitle('Sensitivity Study: Overshoot Percentage vs. Rise Time');
sg21.Color = 'w';

% Bottom: table axes occupying lower third
ax_tbl = axes('Parent', fig21, ...
              'Position', [0.01, 0.01, 0.98, 0.36], ...
              'XLim', [0 1], 'YLim', [0 1], ...
              'Color', 'k', 'XColor', 'k', 'YColor', 'k', ...
              'TickLength', [0 0]);
hold(ax_tbl, 'on');
axis(ax_tbl, 'off');

% Table layout in data coords [0,1]
col_cx   = [0.09, 0.22, 0.31, 0.42, 0.53, 0.67, 0.81];
header_y = 0.88;
row_h    = 0.155;

% Table title
text(ax_tbl, 0.5, 0.97, 'Rise Time Sensitivity  -  % Increase per 5% Overshoot Step', ...
    'HorizontalAlignment', 'center', ...
    'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'Courier New');

% Header background + text
rectangle(ax_tbl, 'Position', [0.005, header_y - row_h*0.78, 0.99, row_h*0.82], ...
    'FaceColor', [0.18 0.18 0.18], 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1);
for c = 1:7
    text(ax_tbl, col_cx(c), header_y - row_h*0.30, col_headers{c}, ...
        'HorizontalAlignment', 'center', ...
        'Color', [1.0 0.85 0.2], 'FontSize', 8.5, ...
        'FontWeight', 'bold', 'FontName', 'Courier New');
end

% Data rows
for r = 1:n
    y_top_row = header_y - r * row_h;
    bg = [0.09 0.09 0.09] * (1 + mod(r,2)*0.5);
    rectangle(ax_tbl, 'Position', [0.005, y_top_row - row_h*0.78, 0.99, row_h*0.82], ...
        'FaceColor', bg, 'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 0.5);

    for c = 1:7
        txt = table_str{r,c};
        if c >= 6 && r > 1
            num_val = str2double(strrep(strrep(txt,'%',''),'+',''));
            if ~isnan(num_val) && num_val > 0;      tc = [0.3 1.0 0.3];
            elseif ~isnan(num_val) && num_val < 0;  tc = [1.0 0.4 0.4];
            else;                                    tc = 'w';
            end
        elseif c == 1;  tc = [0.55 0.85 1.0];
        else;           tc = 'w';
        end
        text(ax_tbl, col_cx(c), y_top_row - row_h*0.30, txt, ...
            'HorizontalAlignment', 'center', ...
            'Color', tc, 'FontSize', 8.5, ...
            'FontWeight', 'normal', 'FontName', 'Courier New');
    end
end

% Bottom divider
bottom_y = header_y - (n+1)*row_h + row_h*0.05;
line(ax_tbl, [0.005, 0.995], [bottom_y, bottom_y], ...
    'Color', [0.5 0.5 0.5], 'LineWidth', 1);

hold(ax_tbl, 'off');