%% LATERAL LOAD TRANSFER ANALYSIS
clear; clc; close all;
paramR26;

% =========================================================================
%% CONSTANTS & PARAMETERS
% =========================================================================
LAT_G        = 1;
ROLL_INERTIA = 182.24965;
TIME_SPAN    = [0 1];
INITIAL_STATE = [0 0 0 0];

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

roll_front_deg = x(:,1) * 180/pi;
roll_rear_deg  = x(:,3) * 180/pi;
omega_front    = x(:,2);
omega_rear     = x(:,4);

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

GLT_f = (vehicle.mass * vehicle.wgt_dist       * 9.81 * LAT_G * suspension.RC_f) / vehicle.track;
GLT_r = (vehicle.mass * (1 - vehicle.wgt_dist) * 9.81 * LAT_G * suspension.RC_r) / vehicle.track;

GLT_f_vec = GLT_f * ones(n, 1);
GLT_r_vec = GLT_r * ones(n, 1);

ELT_f_spring = x(:,1) * suspension.kf;
ELT_f_damper = x(:,2) * suspension.cf;
ELT_r_spring = x(:,3) * suspension.kr;
ELT_r_damper = x(:,4) * suspension.cr;

TLT_f = ELT_f_spring + ELT_f_damper + GLT_f_vec;
TLT_r = ELT_r_spring + ELT_r_damper + GLT_r_vec;

Total_LLT = (LAT_G * 9.81 * vehicle.mass * vehicle.cgh / vehicle.track) * ones(n, 1);

v_damper_f = omega_front * (vehicle.track/2) * vehicle.MR_f * 1000;
v_damper_r = omega_rear  * (vehicle.track/2) * vehicle.MR_r * 1000;

% =========================================================================
%% PLOTS
% =========================================================================

% --- Figure 1: Roll Angles ---
fig1 = figure(1); set_dark_figure(fig1);
ax1  = gca;
plot(ax1, t, roll_front_deg, 'Color', [0.00 0.78 1.00], 'LineWidth', 2.0, 'DisplayName', 'Front'); hold(ax1,'on');
plot(ax1, t, roll_rear_deg,  'Color', [0.45 1.00 0.30], 'LineWidth', 2.0, 'DisplayName', 'Rear');
yline(roll_ss(1), '--', 'Color', [1.00 0.82 0.10], 'LineWidth', 1.4, 'DisplayName', 'Front SS', 'Parent', ax1);
yline(roll_ss(2), '--', 'Color', [1.00 0.82 0.10], 'LineWidth', 1.4, 'DisplayName', 'Rear SS',  'Parent', ax1);
xlabel(ax1,'Time (s)'); ylabel(ax1,'Roll Angle (deg)');
title(ax1,'Roll Angles vs Time'); grid(ax1,'on');
lg1 = legend(ax1,'Location','southeast');
set_dark_axes(ax1); set_dark_legend(lg1);

% --- Figure 2: Roll Angular Velocities ---
fig2 = figure(2); set_dark_figure(fig2);
ax2  = gca;
plot(ax2, t, x(:,2)*180/pi, 'Color', [0.00 0.78 1.00], 'LineWidth', 2.0, 'DisplayName', 'Front'); hold(ax2,'on');
plot(ax2, t, x(:,4)*180/pi, 'Color', [0.45 1.00 0.30], 'LineWidth', 2.0, 'DisplayName', 'Rear');
xlabel(ax2,'Time (s)'); ylabel(ax2,'Angular Velocity (deg/s)');
title(ax2,'Roll Angular Velocities vs Time'); grid(ax2,'on');
lg2 = legend(ax2,'Location','northeast');
set_dark_axes(ax2); set_dark_legend(lg2);

% --- Figure 3: Damper Linear Velocities ---
fig3 = figure(3); set_dark_figure(fig3);
ax3  = gca;
plot(ax3, t, v_damper_f, 'Color', [0.00 0.78 1.00], 'LineWidth', 2.0, 'DisplayName', 'Front'); hold(ax3,'on');
plot(ax3, t, v_damper_r, 'Color', [0.45 1.00 0.30], 'LineWidth', 2.0, 'DisplayName', 'Rear');
xlabel(ax3,'Time (s)'); ylabel(ax3,'Damper Velocity (mm/s)');
title(ax3,'Damper Linear Velocity from Roll'); grid(ax3,'on');
lg3 = legend(ax3,'Location','northeast');
set_dark_axes(ax3); set_dark_legend(lg3);

% --- Figures 4 & 5: Load Transfer Breakdowns ---
plot_load_transfer(t, ELT_f_spring, ELT_f_damper, GLT_f_vec, TLT_f, 'Front');
plot_load_transfer(t, ELT_r_spring, ELT_r_damper, GLT_r_vec, TLT_r, 'Rear');

% --- Figure 6: Total LLT Verification ---
fig6 = figure(6); set_dark_figure(fig6);
ax6  = gca;
plot(ax6, t, TLT_f,         'Color', [0.00 0.78 1.00], 'LineWidth', 2.0, 'DisplayName', 'Front Total');   hold(ax6,'on');
plot(ax6, t, TLT_r,         'Color', [0.45 1.00 0.30], 'LineWidth', 2.0, 'DisplayName', 'Rear Total');
plot(ax6, t, TLT_f + TLT_r, 'Color', [1.00 0.82 0.10], 'LineWidth', 2.0, 'DisplayName', 'Combined Total');
plot(ax6, t, Total_LLT,     'Color', [1.00 0.45 0.05], 'LineWidth', 1.6, 'LineStyle', '--', 'DisplayName', 'Vehicle Total');
xlabel(ax6,'Time (s)'); ylabel(ax6,'Load Transfer (N)');
title(ax6,'Total Lateral Load Transfer Verification'); grid(ax6,'on');
lg6 = legend(ax6,'Location','southeast');
set_dark_axes(ax6); set_dark_legend(lg6);

% =========================================================================
%% FIGURE 7: SUMMARY TABLES
% =========================================================================

[peak_roll_f,  ~] = max(abs(roll_front_deg));
[peak_roll_r,  ~] = max(abs(roll_rear_deg));
[peak_omega_f, ~] = max(abs(x(:,2)*180/pi));
[peak_omega_r, ~] = max(abs(x(:,4)*180/pi));
[peak_vdamp_f, ~] = max(abs(v_damper_f));
[peak_vdamp_r, ~] = max(abs(v_damper_r));
[peak_ELTs_f,  ~] = max(abs(ELT_f_spring));
[peak_ELTs_r,  ~] = max(abs(ELT_r_spring));
[peak_ELTd_f,  ~] = max(abs(ELT_f_damper));
[peak_ELTd_r,  ~] = max(abs(ELT_r_damper));
[peak_TLT_f,   ~] = max(abs(TLT_f));
[peak_TLT_r,   ~] = max(abs(TLT_r));

ss_idx     = round(0.95 * length(t)):length(t);
ss_roll_f  = mean(roll_front_deg(ss_idx));
ss_roll_r  = mean(roll_rear_deg(ss_idx));
ss_omega_f = mean(x(ss_idx,2)*180/pi);
ss_omega_r = mean(x(ss_idx,4)*180/pi);
ss_vdamp_f = mean(v_damper_f(ss_idx));
ss_vdamp_r = mean(v_damper_r(ss_idx));
ss_ELTs_f  = mean(ELT_f_spring(ss_idx));
ss_ELTs_r  = mean(ELT_r_spring(ss_idx));
ss_ELTd_f  = mean(ELT_f_damper(ss_idx));
ss_ELTd_r  = mean(ELT_r_damper(ss_idx));
ss_TLT_f   = mean(TLT_f(ss_idx));
ss_TLT_r   = mean(TLT_r(ss_idx));

roll_grad_f       = roll_ss(1) / LAT_G;
roll_grad_r       = roll_ss(2) / LAT_G;
TLT_f_theoretical = GLT_f + roll_ss(1)*(pi/180) * suspension.kf;
TLT_r_theoretical = GLT_r + roll_ss(2)*(pi/180) * suspension.kr;

% ---- figure ----
fig7 = figure(7); set_dark_figure(fig7);
set(fig7, 'Position', [100 100 1300 750]);

BG      = [0.05  0.06  0.10];
AX_DARK = [0.07  0.09  0.14];
HDR_COL = [0.12  0.16  0.26];
ALT_COL = [0.09  0.12  0.20];
TXT_W   = [0.92  0.92  0.92];
TXT_Y   = [1.00  0.82  0.10];
TXT_G   = [0.45  1.00  0.30];

ax7 = axes(fig7, 'Position', [0 0 1 1]);
set(ax7, 'Color', BG, 'XColor', BG, 'YColor', BG, ...
    'XLim', [0 1], 'YLim', [0 1], 'XTick', [], 'YTick', []);
axis(ax7, 'off');

    function draw_cell(ax, x, y, w, h, bgcol, txt, txtcol, fs, bold)
        EC = [0.25 0.30 0.45];
        rectangle(ax, 'Position', [x y w h], ...
            'FaceColor', bgcol, 'EdgeColor', EC, 'LineWidth', 0.6);
        fw = 'normal'; if bold, fw = 'bold'; end
        text(ax, x + w/2, y + h/2, txt, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'Color', txtcol, 'FontSize', fs, 'FontName', 'Arial', 'FontWeight', fw);
    end

    function s = fmt_delta(peak_val, ss_val)
        d = peak_val - ss_val;
        if d >= 0, s = sprintf('+%.3g', d);
        else,      s = sprintf('%.3g',  d); end
    end

    function c = delta_col(peak_val, ss_val, base_col)
        if abs(peak_val - ss_val) > 0.10 * abs(peak_val)
            c = [1.00 0.55 0.10];
        else
            c = base_col;
        end
    end

% =========================================================================
%  TABLE 1 — left half  [0.01 … 0.49]
% =========================================================================
t1_y0  = 0.08;
row_h  = 0.072;
n_rows = 9;

% Each column: [start_x, width]  — right edge of last = 0.010+0.150+0.055+0.055+0.050+0.055+0.055+0.050 = 0.480
c1x = 0.010;  c1w = 0.150;
c2x = 0.160;  c2w = 0.055;
c3x = 0.215;  c3w = 0.055;
c4x = 0.270;  c4w = 0.050;
c5x = 0.320;  c5w = 0.055;
c6x = 0.375;  c6w = 0.055;
c7x = 0.430;  c7w = 0.050;
% right edge = 0.430 + 0.050 = 0.480  ✓

col_x1 = [c1x c2x c3x c4x c5x c6x c7x];
col_w1 = [c1w c2w c3w c4w c5w c6w c7w];
t1_total_w = c7x + c7w - c1x;   % 0.470

text(ax7, c1x + t1_total_w/2, t1_y0 + n_rows*row_h + 0.025, ...
    'DYNAMIC RESPONSE SUMMARY', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
    'Color', TXT_W, 'FontSize', 11, 'FontName', 'Arial', 'FontWeight', 'bold');

headers1 = {'Variable','Front Peak','Front SS','\DeltaFront','Rear Peak','Rear SS','\DeltaRear'};
for c = 1:7
    draw_cell(ax7, col_x1(c), t1_y0+(n_rows-1)*row_h, col_w1(c), row_h, HDR_COL, headers1{c}, TXT_W, 11, true);
end

row_labels = { ...
    'Roll Angle  (deg)',       'Roll Rate  (deg/s)',      'Damper Velocity  (mm/s)', ...
    'ELT Spring  (N)',         'ELT Damper  (N)',         'Total LLT  (N)', ...
    'Axle Moment  (N·m)',      'Roll Gradient  (deg/g)'  };

front_peak = [peak_roll_f; peak_omega_f; peak_vdamp_f; peak_ELTs_f; peak_ELTd_f; peak_TLT_f; moment_front; NaN];
front_ss   = [ss_roll_f;   ss_omega_f;   ss_vdamp_f;   ss_ELTs_f;   ss_ELTd_f;   ss_TLT_f;   moment_front; roll_grad_f];
rear_peak  = [peak_roll_r; peak_omega_r; peak_vdamp_r; peak_ELTs_r; peak_ELTd_r; peak_TLT_r; moment_rear;  NaN];
rear_ss    = [ss_roll_r;   ss_omega_r;   ss_vdamp_r;   ss_ELTs_r;   ss_ELTd_r;   ss_TLT_r;   moment_rear;  roll_grad_r];

no_delta_rows = [2, 3, 5, 7, 8];

for r = 1:8
    row_idx = n_rows - 1 - r;
    bg    = AX_DARK; if mod(r,2)==0, bg = ALT_COL; end
    y_row = t1_y0 + row_idx * row_h;

    skip_peak  = isnan(front_peak(r));
    skip_delta = ismember(r, no_delta_rows);

    if skip_peak
        fp_str = '—'; rp_str = '—';
    else
        fp_str = sprintf('%.3g', front_peak(r));
        rp_str = sprintf('%.3g', rear_peak(r));
    end

    if skip_delta || skip_peak
        fd_str = '—'; rd_str = '—';
        fd_col = [0.55 0.58 0.65]; rd_col = [0.55 0.58 0.65];
    else
        fd_str = fmt_delta(front_peak(r), front_ss(r));
        rd_str = fmt_delta(rear_peak(r),  rear_ss(r));
        fd_col = delta_col(front_peak(r), front_ss(r), TXT_Y);
        rd_col = delta_col(rear_peak(r),  rear_ss(r),  TXT_G);
    end

    draw_cell(ax7, col_x1(1), y_row, col_w1(1), row_h, bg, row_labels{r},               TXT_W,  10, false);
    draw_cell(ax7, col_x1(2), y_row, col_w1(2), row_h, bg, fp_str,                       TXT_Y,  10, false);
    draw_cell(ax7, col_x1(3), y_row, col_w1(3), row_h, bg, sprintf('%.3g',front_ss(r)), TXT_Y,  10, false);
    draw_cell(ax7, col_x1(4), y_row, col_w1(4), row_h, bg, fd_str,                       fd_col, 10, false);
    draw_cell(ax7, col_x1(5), y_row, col_w1(5), row_h, bg, rp_str,                       TXT_G,  10, false);
    draw_cell(ax7, col_x1(6), y_row, col_w1(6), row_h, bg, sprintf('%.3g',rear_ss(r)),  TXT_G,  10, false);
    draw_cell(ax7, col_x1(7), y_row, col_w1(7), row_h, bg, rd_str,                       rd_col, 10, false);
end

% =========================================================================
%  TABLE 2 — right half  [0.52 … 0.98]
% =========================================================================
t2_y0    = 0.08;
n_th     = 7;
th_row_h = 0.072;

d1x = 0.520;  d1w = 0.250;
d2x = 0.770;  d2w = 0.140;
d3x = 0.910;  d3w = 0.060;
% right edge = 0.910 + 0.060 = 0.970  ✓

th_col_x   = [d1x d2x d3x];
th_col_w   = [d1w d2w d3w];
th_total_w = d3x + d3w - d1x;   % 0.450

text(ax7, d1x + th_total_w/2, t2_y0 + n_th*th_row_h + 0.025, ...
    'THEORETICAL  (STEADY-STATE, OUTSIDE EOM)', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
    'Color', TXT_W, 'FontSize', 11, 'FontName', 'Arial', 'FontWeight', 'bold');

th_headers = {'Quantity', 'Value', 'Unit'};
for c = 1:3
    draw_cell(ax7, th_col_x(c), t2_y0+(n_th-1)*th_row_h, th_col_w(c), th_row_h, HDR_COL, th_headers{c}, TXT_W, 11, true);
end

th_data = { ...
    'SS Roll Angle — Front',   sprintf('%.4f', roll_ss(1)),        'deg', TXT_Y; ...
    'SS Roll Angle — Rear',    sprintf('%.4f', roll_ss(2)),        'deg', TXT_G; ...
    'Geometric LLT — Front',   sprintf('%.2f', GLT_f),             'N',   TXT_Y; ...
    'Geometric LLT — Rear',    sprintf('%.2f', GLT_r),             'N',   TXT_G; ...
    'Total LLT — Front (th.)', sprintf('%.2f', TLT_f_theoretical), 'N',   TXT_Y; ...
    'Total LLT — Rear  (th.)', sprintf('%.2f', TLT_r_theoretical), 'N',   TXT_G; ...
};

for r = 1:6
    bg    = AX_DARK; if mod(r,2)==0, bg = ALT_COL; end
    y_row = t2_y0 + (n_th-1-r)*th_row_h;
    draw_cell(ax7, th_col_x(1), y_row, th_col_w(1), th_row_h, bg, th_data{r,1}, TXT_W,        10, false);
    draw_cell(ax7, th_col_x(2), y_row, th_col_w(2), th_row_h, bg, th_data{r,2}, th_data{r,4}, 10, false);
    draw_cell(ax7, th_col_x(3), y_row, th_col_w(3), th_row_h, bg, th_data{r,3}, TXT_W,        10, false);
end

% =========================================================================
%  Super-title and info strip
% =========================================================================
text(ax7, 0.5, 0.975, ...
    sprintf('Lateral Load Transfer — Results Summary  |  %.1f g lateral', LAT_G), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'Color', TXT_W, 'FontSize', 13, 'FontName', 'Arial', 'FontWeight', 'bold');

info_str = sprintf( ...
    'Front:  \\omega_n = %.2f rad/s  (%.2f Hz)  |  \\zeta = %.3f          Rear:  \\omega_n = %.2f rad/s  (%.2f Hz)  |  \\zeta = %.3f', ...
    omega_nf, omega_nf/(2*pi), zeta_f, omega_nr, omega_nr/(2*pi), zeta_r);
text(ax7, 0.5, 0.930, info_str, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'Color', [0.65 0.70 0.85], 'FontSize', 9, 'FontName', 'Arial');

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
    fig = figure; set_dark_figure(fig);
    ax  = gca;
    plot(ax, t, ELT_spring, 'Color', [0.00 0.78 1.00], 'LineWidth', 2.0, 'DisplayName', 'Elastic Spring'); hold(ax,'on');
    plot(ax, t, ELT_damper, 'Color', [0.45 1.00 0.30], 'LineWidth', 2.0, 'DisplayName', 'Elastic Damper');
    plot(ax, t, GLT,        'Color', [1.00 0.82 0.10], 'LineWidth', 2.0, 'DisplayName', 'Geometric');
    plot(ax, t, TLT,        'Color', [1.00 0.45 0.05], 'LineWidth', 2.2, 'LineStyle', '--', 'DisplayName', 'Total');
    xlabel(ax,'Time (s)'); ylabel(ax,'Load Transfer (N)');
    title(ax, sprintf('%s Axle — Load Transfer Breakdown', location)); grid(ax,'on');
    lg = legend(ax,'Location','southeast');
    set_dark_axes(ax); set_dark_legend(lg);
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

function set_dark_figure(fig)
    fig.Color = [0.05 0.06 0.10];
end

function set_dark_axes(ax)
    AX_DARK  = [0.07  0.09  0.14];
    GRID_COL = [0.18  0.22  0.32];
    TEXT_COL = [0.92  0.92  0.92];
    TICK_COL = [0.75  0.75  0.80];

    ax.Color              = AX_DARK;
    ax.XColor             = TICK_COL;
    ax.YColor             = TICK_COL;
    ax.ZColor             = TICK_COL;
    ax.GridColor          = GRID_COL;
    ax.GridAlpha          = 1.0;
    ax.GridLineStyle      = '-';
    ax.MinorGridColor     = GRID_COL;
    ax.MinorGridAlpha     = 0.5;
    ax.MinorGridLineStyle = ':';
    ax.XMinorGrid         = 'on';
    ax.YMinorGrid         = 'on';
    ax.Title.Color        = TEXT_COL;
    ax.Title.FontWeight   = 'bold';
    ax.Title.FontSize     = 11;
    ax.XLabel.Color       = TEXT_COL;
    ax.YLabel.Color       = TEXT_COL;
    ax.XLabel.FontSize    = 10;
    ax.YLabel.FontSize    = 10;
    ax.TickDir            = 'out';
    ax.TickLength         = [0.008 0.02];
    ax.LineWidth          = 0.8;
    ax.FontSize           = 9;
    ax.FontName           = 'Arial';
    ax.Box                = 'off';
end

function set_dark_legend(lg)
    lg.Color     = [0.07 0.09 0.14];
    lg.TextColor = [0.92 0.92 0.92];
    lg.EdgeColor = [0.30 0.35 0.50];
    lg.FontSize  = 9;
    lg.FontName  = 'Arial';
end