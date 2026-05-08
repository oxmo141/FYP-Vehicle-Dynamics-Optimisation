%% ============================================================
%  Front Wing Ground Clearance — Pitch Motion Study
%  Reference point: 870.20 mm forward of front axle, 47 mm above ground
%
%  PIVOT: Centre of Gravity (CG)
%
%  GEOMETRY:
%    The car pitches about the CG.  The front axle sits a distance
%    car.a ahead of the CG, so it drops when the nose pitches down:
%
%      z_fa  = z_CG - a*sin(theta)          [front axle height above ground]
%
%    The wing reference point is x_ref ahead of the front axle, so its
%    total distance from the CG is (a + x_ref).  Its height is:
%
%      z_tip = z_CG - a*sin(th) + z_ref*cos(th) - x_ref*sin(th)
%            = z_CG - (a + x_ref)*sin(th) + z_ref*cos(th)
%
%    Drop due to pitch alone (z_CG = 0):
%      delta_z_pitch = (a + x_ref)*sin(th) - z_ref*(1 - cos(th))
%
%    Clearance:
%      delta_z = z_tip   [positive = above ground, negative = contact]
%
%  NOTE: heave_cg_mm in the function signature is z_CG (heave at CG,
%        not at the front axle).  The CG-to-front-axle geometry is
%        handled inside wingTipClearance via a_mm.
%% ============================================================
clear; clc; close all;
paramR26

%% ---- Vehicle Geometry from paramR26 ----------------------------
a_m = car.a;            % CG to front axle  (metres)
b_m = car.b;            % CG to rear  axle  (metres)
a   = a_m * 1e3;        % convert to mm
b   = b_m * 1e3;        % convert to mm
L   = a + b;            % wheelbase (mm)

%% ---- Reference Point Geometry ----------------------------------
x_ref = 870.20;         % mm  forward of front axle (+ve = forward)
z_ref = 47.00;          % mm  above ground, static ride height

fprintf('=== Front Wing Pitch Clearance Analysis ===\n');
fprintf('Pivot              : CG\n');
fprintf('CG to front axle a : %.2f mm\n', a);
fprintf('CG to rear  axle b : %.2f mm\n', b);
fprintf('Wheelbase L        : %.2f mm\n', L);
fprintf('Reference point    : %.2f mm ahead of front axle\n', x_ref);
fprintf('Reference point    : %.2f mm ahead of CG  (= a + x_ref)\n', a + x_ref);
fprintf('Static height      : %.2f mm above ground\n\n', z_ref);

%% ---- Primary Study: 1 degree of pitch, zero CG heave -----------
theta_study_deg = 1.0;
heave_cg_mm     = 0.0;   % z_CG from ride model (mm); set to peak_cg_mm here

[z_tip_study, drop_study, z_fa_study] = wingTipClearance( ...
    theta_study_deg, heave_cg_mm, a, x_ref, z_ref);

fprintf('--- Study Point: theta = %.2f deg, z_CG = %.2f mm ---\n', ...
        theta_study_deg, heave_cg_mm);
fprintf('Front axle drop a*sin(th)        : %.4f mm\n', a * sind(theta_study_deg));
fprintf('Additional drop  x_ref*sin(th)   : %.4f mm\n', x_ref * sind(theta_study_deg));
fprintf('Total tip drop (pitch only)      : %.4f mm\n', drop_study);
fprintf('Front axle height z_fa           : %.4f mm\n', z_fa_study);
fprintf('Remaining clearance              : %.4f mm\n', z_tip_study);
if z_tip_study > 15
    fprintf('Status                           : CLEAR (>15 mm margin)\n\n');
elseif z_tip_study > 0
    fprintf('Status                           : LOW MARGIN (<15 mm)\n\n');
else
    fprintf('Status                           : GROUND CONTACT\n\n');
end

%% ---- Sensitivity at study point --------------------------------
dth        = 1e-4;   % deg
[z1, ~, ~] = wingTipClearance(theta_study_deg + dth, heave_cg_mm, a, x_ref, z_ref);
[z0, ~, ~] = wingTipClearance(theta_study_deg,        heave_cg_mm, a, x_ref, z_ref);
sens_theta = (z1 - z0) / dth;   % mm per deg

fprintf('--- Sensitivity at study point ---\n');
fprintf('dClearance/dTheta          : %.4f mm/deg\n', sens_theta);
fprintf('Effective moment arm       : %.2f mm  (= a + x_ref)\n', a + x_ref);
fprintf('dClearance/dz_CG           : 1.0000 mm/mm  (unity)\n\n');

%% ---- Critical pitch angle (z_CG = 0) ---------------------------
%  Solve: -(a+x_ref)*sin(th) + z_ref*cos(th) = 0
%  Analytical: th_crit = atan2(z_ref, a+x_ref)
theta_crit_exact = atan2d(z_ref, a + x_ref);
fprintf('Critical pitch (analytical, z_CG=0): %.4f deg\n\n', theta_crit_exact);
theta_crit = theta_crit_exact;

%% ---- Sweep Table: -3 to +3 deg, z_CG = 0 ----------------------
theta_sweep = (-3 : 1 : 3)';
fprintf('--- Clearance Sweep (z_CG = 0 mm) ---\n');
fprintf('%-10s %-20s %-18s %-18s %-14s %-12s\n', ...
        'theta(deg)', 'Drop_total(mm)', 'FA_drop(mm)', ...
        'FA_height(mm)', 'Clearance(mm)', 'Status');
fprintf('%s\n', repmat('-', 1, 95));
for i = 1 : length(theta_sweep)
    [zt, dp, z_fa_i] = wingTipClearance(theta_sweep(i), 0, a, x_ref, z_ref);
    fa_drop = a * sind(theta_sweep(i));
    if zt > 15
        st = 'CLEAR';
    elseif zt > 0
        st = 'LOW MARGIN';
    else
        st = 'CONTACT';
    end
    marker = '';
    if theta_sweep(i) == theta_study_deg
        marker = ' <-- study point';
    end
    fprintf('%-+10.2f %-20.4f %-18.4f %-18.4f %-14.4f %s%s\n', ...
            theta_sweep(i), dp, fa_drop, z_fa_i, zt, st, marker);
end
fprintf('\n');

%% ---- Pitch sweep vectors for plotting --------------------------
theta_vec   = linspace(-5, 5, 500);
clr_vec     = zeros(size(theta_vec));
drop_vec    = zeros(size(theta_vec));
z_fa_vec    = zeros(size(theta_vec));
for i = 1 : length(theta_vec)
    [clr_vec(i), drop_vec(i), z_fa_vec(i)] = ...
        wingTipClearance(theta_vec(i), 0, a, x_ref, z_ref);
end

%% ---- CG heave + pitch combined grid ----------------------------
theta_grid = linspace(-5, 5,   200);
heave_grid = linspace(-50, 50, 200);   % z_CG range (mm)
[TH, HV]   = meshgrid(theta_grid, heave_grid);
CLR        = zeros(size(TH));
for i = 1 : numel(TH)
    [CLR(i), ~, ~] = wingTipClearance(TH(i), HV(i), a, x_ref, z_ref);
end

%% ============================================================
%  PLOTTING
%% ============================================================
col_ok     = [0.11  0.62  0.46];
col_warn   = [0.73  0.46  0.04];
col_danger = [0.64  0.18  0.18];
col_line   = [0.09  0.37  0.65];
col_study  = [0.55  0.20  0.70];
col_fa     = [0.95  0.55  0.10];
col_grid   = [0.25  0.25  0.30];
col_txt    = [0.92  0.92  0.92];
col_bg     = [0.05  0.05  0.08];
col_ax     = [0.08  0.08  0.12];

fig = figure('Name', 'Front Wing Pitch Clearance — CG Pivot', ...
             'Color', col_bg, ...
             'Position', [60 50 1380 940]);

lm      = 0.055;  rm = 0.020;
col_gap = 0.035;
col_w   = (1 - lm - rm - col_gap) / 2;

%% ---- Plot 1: Clearance vs pitch angle (top-left) ---------------
ax1 = axes('Position', [lm  0.57  col_w  0.37]);
hold on;

ymin_ax = min(clr_vec) - 5;
ymax_ax = max(clr_vec) + 5;

fill([theta_vec(1) theta_vec(end) theta_vec(end) theta_vec(1)], ...
     [ymin_ax ymin_ax 0 0], col_danger, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
fill([theta_vec(1) theta_vec(end) theta_vec(end) theta_vec(1)], ...
     [0 0 15 15], col_warn, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
fill([theta_vec(1) theta_vec(end) theta_vec(end) theta_vec(1)], ...
     [15 15 ymax_ax ymax_ax], col_ok, 'FaceAlpha', 0.10, 'EdgeColor', 'none');

yline(0,  '--', 'Color', [col_danger 0.7], 'LineWidth', 0.9, ...
      'Label', 'Ground contact', 'LabelHorizontalAlignment', 'left', ...
      'FontColor', col_txt, 'FontSize', 8);
yline(15, '--', 'Color', [col_warn 0.7], 'LineWidth', 0.9, ...
      'Label', '15 mm margin', 'LabelHorizontalAlignment', 'left', ...
      'FontColor', col_txt, 'FontSize', 8);

plot(theta_vec, z_fa_vec, ':', 'Color', [col_fa 0.8], 'LineWidth', 1.2, ...
     'DisplayName', 'Front axle height z_{fa}  (= z_{CG} - a\cdotsin\theta)');
plot(theta_vec, clr_vec, 'Color', col_line, 'LineWidth', 2.0, ...
     'DisplayName', 'Wing tip clearance \Deltaz');

plot(theta_study_deg, z_tip_study, 'o', ...
     'MarkerSize', 8, 'MarkerFaceColor', col_study, ...
     'MarkerEdgeColor', 'w', 'LineWidth', 1.2, ...
     'DisplayName', sprintf('Study: \\theta=%.1f°, \\Deltaz=%.2f mm', ...
     theta_study_deg, z_tip_study));

xline(theta_crit, '--', 'Color', [col_danger 0.8], 'LineWidth', 1.0, ...
      'Label', sprintf('\\theta_{crit}=%.3f°', theta_crit), ...
      'LabelHorizontalAlignment', 'right', ...
      'FontColor', col_txt, 'FontSize', 8);

ylim([ymin_ax ymax_ax]);
xlabel('Pitch angle \theta (deg)',  'Color', col_txt, 'FontSize', 11);
ylabel('Height / Clearance (mm)',   'Color', col_txt, 'FontSize', 11);
title('Wing Clearance vs Pitch Angle  (z_{CG} = 0,  pivot = CG)', ...
      'Color', col_txt, 'FontSize', 12, 'FontWeight', 'bold');
legend('Contact zone', 'Low margin', 'Clear zone', ...
       'Front axle height', 'Wing tip clearance', ...
       sprintf('Study: %.1f° \\rightarrow %.2f mm', theta_study_deg, z_tip_study), ...
       'Location', 'northeast', ...
       'TextColor', col_txt, 'Color', col_ax, 'EdgeColor', col_grid, 'FontSize', 8);
applyDarkAxes(ax1, col_grid, col_txt);

%% ---- Plot 2: Drop components (top-right) -----------------------
ax2 = axes('Position', [lm+col_w+col_gap  0.57  col_w  0.37]);
hold on;

drop_fa_vec  = a     .* sind(theta_vec);
drop_ref_vec = x_ref .* sind(theta_vec);
drop_z2_vec  = z_ref .* (1 - cosd(theta_vec));

% Representative combined case — set to peak_cg_mm from your RK4 results
example_heave_cg = 10;   % mm  (edit this)
clr_combined = zeros(size(theta_vec));
for i = 1 : length(theta_vec)
    [clr_combined(i), ~, ~] = ...
        wingTipClearance(theta_vec(i), -example_heave_cg, a, x_ref, z_ref);
end

plot(theta_vec, drop_fa_vec,  ':', 'Color', col_fa,           'LineWidth', 1.5, ...
     'DisplayName', sprintf('FA drop  a\\cdotsin\\theta  (a=%.0f mm)', a));
plot(theta_vec, drop_ref_vec, ':', 'Color', [0.20 0.75 0.95], 'LineWidth', 1.5, ...
     'DisplayName', sprintf('x_{ref}\\cdotsin\\theta  (x_{ref}=%.0f mm)', x_ref));
plot(theta_vec, drop_fa_vec + drop_ref_vec - drop_z2_vec, '-', ...
     'Color', col_line, 'LineWidth', 2.0, ...
     'DisplayName', 'Total drop  (a+x_{ref})\\cdotsin\\theta');
plot(theta_vec, clr_combined, '--', 'Color', col_warn, 'LineWidth', 1.8, ...
     'DisplayName', sprintf('Clearance (z_{CG}=−%.0f mm)', example_heave_cg));

yline(z_ref, ':', 'Color', [col_txt 0.30], 'LineWidth', 0.8, ...
      'Label', sprintf('z_{ref}=%.0f mm', z_ref), ...
      'LabelHorizontalAlignment', 'right', 'FontColor', col_txt, 'FontSize', 8);
yline(0, '--', 'Color', [col_danger 0.7], 'LineWidth', 0.9);

xlabel('Pitch angle \theta (deg)', 'Color', col_txt, 'FontSize', 11);
ylabel('Drop / Displacement (mm)', 'Color', col_txt, 'FontSize', 11);
title(sprintf('Drop Components — pivot at CG  (a = %.0f mm,  b = %.0f mm)', a, b), ...
      'Color', col_txt, 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'TextColor', col_txt, ...
       'Color', col_ax, 'EdgeColor', col_grid, 'FontSize', 8);
applyDarkAxes(ax2, col_grid, col_txt);

%% ---- Plot 3: Contour map — z_CG vs pitch (bottom-left) ---------
ax3 = axes('Position', [lm  0.07  col_w  0.42]);
contourf(TH, HV, CLR, 30, 'LineColor', 'none');

colormap(ax3, customClearanceColormap(256));
cb = colorbar(ax3);
cb.Color = col_txt;  cb.FontSize = 9;
ylabel(cb, 'Clearance \Delta z (mm)', 'Color', col_txt, 'FontSize', 10);

hold on;
contour(TH, HV, CLR, [0 0], 'LineColor', col_danger, 'LineWidth', 2.0, ...
        'DisplayName', 'Contact boundary (\Deltaz = 0)');
contour(TH, HV, CLR, [15 15], '--', 'LineColor', col_warn, 'LineWidth', 1.5, ...
        'DisplayName', '15 mm margin');
plot(theta_study_deg, heave_cg_mm, 'p', ...
     'MarkerSize', 12, 'MarkerFaceColor', col_study, 'MarkerEdgeColor', 'w', ...
     'LineWidth', 1.2, ...
     'DisplayName', sprintf('Study pt (%.1f°, z_{CG}=%.1f mm)', ...
     theta_study_deg, heave_cg_mm));

xlabel('Pitch angle \theta (deg)', 'Color', col_txt, 'FontSize', 11);
ylabel('CG heave z_{CG} (mm)',     'Color', col_txt, 'FontSize', 11);
title('Clearance Contour — z_{CG} vs Pitch  (pivot = CG)', ...
      'Color', col_txt, 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'TextColor', col_txt, ...
       'Color', col_ax, 'EdgeColor', col_grid, 'FontSize', 8);
applyDarkAxes(ax3, col_grid, col_txt);

%% ---- Plot 4: Geometry schematic (bottom-right) -----------------
ax4 = axes('Position', [lm+col_w+col_gap  0.07  col_w  0.42]);
hold on;

theta_s = theta_study_deg;

% Canvas scale: mm -> canvas units
sc    = 0.38;
cg_y  = 60;       % CG canvas height above ground line

fa_cx = -a    * sc;           % front axle x (unrotated)
ra_cx =  b    * sc;           % rear  axle x
tip_cx = -(a + x_ref) * sc;  % wing tip  x
tip_cy =  z_ref * sc;        % wing tip  y above axle ground level

% Ground
plot([tip_cx - 25,  ra_cx + 35], [0 0], '-', ...
     'Color', [col_txt 0.25], 'LineWidth', 1.0);
text(ra_cx + 37, 0, 'Ground', 'Color', [col_txt 0.5], 'FontSize', 8);

% Rotation matrix (nose-down = +theta, pivot at CG)
R = [ cosd(theta_s)  sind(theta_s);
     -sind(theta_s)  cosd(theta_s)];

% Ghost: static car outline
car_h_c = 18;
ghost = [fa_cx  ra_cx  ra_cx  fa_cx;
          0      0      car_h_c  car_h_c];
ghost(2,:) = ghost(2,:) + cg_y;
patch(ghost(1,:), ghost(2,:), 'none', ...
      'EdgeColor', [col_txt 0.18], 'LineStyle', ':', ...
      'FaceColor', 'none', 'LineWidth', 0.7);

% Ghost wing tip
plot(tip_cx, tip_cy + cg_y, 'v', 'MarkerSize', 6, ...
     'MarkerFaceColor', 'none', 'MarkerEdgeColor', [col_txt 0.25], 'LineWidth', 0.7);
plot([tip_cx tip_cx], [0 tip_cy + cg_y], ':', ...
     'Color', [col_txt 0.15], 'LineWidth', 0.7);

% Pitched body: corners relative to CG at origin, then rotate, then shift
bdy_rel = [fa_cx   ra_cx   ra_cx         fa_cx;
            -cg_y   -cg_y   car_h_c-cg_y  car_h_c-cg_y];
bdy_rot = R * bdy_rel;
bdy_rot(1,:) = bdy_rot(1,:) + 0;     % CG stays at x=0
bdy_rot(2,:) = bdy_rot(2,:) + cg_y;
patch(bdy_rot(1,:), bdy_rot(2,:), [0.18 0.18 0.22], ...
      'EdgeColor', col_txt, 'LineWidth', 0.8, 'FaceAlpha', 0.75);

% CG marker (pivot — fixed in space)
plot(0, cg_y, '+', 'MarkerSize', 11, 'Color', [0.55 0.85 1.0], 'LineWidth', 2.0);
text(4, cg_y + 7, 'CG (pivot)', 'Color', [0.55 0.85 1.0], 'FontSize', 8);

% Front axle (rotated)
fa_rel = R * [fa_cx; -cg_y];
fa_abs = [fa_rel(1); fa_rel(2) + cg_y];
viscircles(fa_abs', 11, 'Color', col_fa, 'LineWidth', 0.9);
text(fa_abs(1) - 2, fa_abs(2) - 18, 'Front axle', ...
     'Color', col_fa, 'FontSize', 7, 'HorizontalAlignment', 'center');

% Rear axle (rotated)
ra_rel = R * [ra_cx; -cg_y];
ra_abs = [ra_rel(1); ra_rel(2) + cg_y];
viscircles(ra_abs', 11, 'Color', [col_txt 0.4], 'LineWidth', 0.9);
text(ra_abs(1), ra_abs(2) - 18, 'Rear axle', ...
     'Color', [col_txt 0.5], 'FontSize', 7, 'HorizontalAlignment', 'center');

% Wing tip (rotated about CG)
tip_rel_cg = [tip_cx; tip_cy - cg_y];
tip_rot    = R * tip_rel_cg;
tip_abs    = [tip_rot(1); tip_rot(2) + cg_y];

clr_col = col_ok;
if z_tip_study <= 0;  clr_col = col_danger;  end

plot([tip_abs(1) tip_abs(1)], [0 tip_abs(2)], '--', ...
     'Color', clr_col, 'LineWidth', 1.5);
plot(tip_abs(1), tip_abs(2), 'v', 'MarkerSize', 10, ...
     'MarkerFaceColor', clr_col, 'MarkerEdgeColor', 'w', 'LineWidth', 1.2);
text(tip_abs(1) + 5, tip_abs(2)/2, ...
     sprintf('\\Deltaz = %.2f mm', z_tip_study), ...
     'Color', clr_col, 'FontSize', 9, 'FontWeight', 'bold');

% Pitch arc at CG
ang_arc = linspace(-90, -90 + theta_s, 40);
arc_r   = 38;
plot(arc_r * cosd(ang_arc), arc_r * sind(ang_arc) + cg_y, ...
     'Color', [0.55 0.85 1.0], 'LineWidth', 1.2);
text(arc_r * 0.6, cg_y + 10, sprintf('\\theta = %.1f°', theta_s), ...
     'Color', [0.55 0.85 1.0], 'FontSize', 8);

% Dimension: a (CG to front axle)
y_d1 = -28;
plot([0 fa_cx],     [y_d1 y_d1], '-', 'Color', [col_fa 0.7], 'LineWidth', 0.9);
plot([0    0],      [y_d1-4 y_d1+4], '-', 'Color', [col_fa 0.7], 'LineWidth', 0.9);
plot([fa_cx fa_cx], [y_d1-4 y_d1+4], '-', 'Color', [col_fa 0.7], 'LineWidth', 0.9);
text(fa_cx/2, y_d1 - 9, sprintf('a = %.0f mm', a), ...
     'Color', col_fa, 'FontSize', 8, 'HorizontalAlignment', 'center');

% Dimension: x_ref (front axle to wing tip, static)
y_d2 = -41;
plot([fa_cx  tip_cx],  [y_d2 y_d2], '-', 'Color', [col_txt 0.45], 'LineWidth', 0.9);
plot([fa_cx  fa_cx],   [y_d2-4 y_d2+4], '-', 'Color', [col_txt 0.45], 'LineWidth', 0.9);
plot([tip_cx tip_cx],  [y_d2-4 y_d2+4], '-', 'Color', [col_txt 0.45], 'LineWidth', 0.9);
text((fa_cx+tip_cx)/2, y_d2 - 9, sprintf('x_{ref} = %.1f mm', x_ref), ...
     'Color', [col_txt 0.65], 'FontSize', 8, 'HorizontalAlignment', 'center');

% z_ref static height indicator
x_arr = tip_cx - 16;
plot([x_arr x_arr], [0, tip_cy + cg_y], '-', 'Color', [col_txt 0.35], 'LineWidth', 0.8);
text(x_arr - 3, (tip_cy + cg_y)*0.5 + cg_y*0.05, ...
     sprintf('z_{ref} = %.0f mm', z_ref), ...
     'Color', [col_txt 0.55], 'FontSize', 8, 'HorizontalAlignment', 'right');

xlim([tip_cx - 45,  ra_cx + 50]);
ylim([-58, cg_y + car_h_c + 32]);
xlabel('Longitudinal (canvas, forward = left)', 'Color', col_txt, 'FontSize', 10);
ylabel('Vertical (canvas units)',               'Color', col_txt, 'FontSize', 10);
title(sprintf(['Geometry Schematic — \\theta = %.1f°,  \\Deltaz = %.2f mm' ...
               '  (pivot = CG,  a = %.0f mm)'], theta_s, z_tip_study, a), ...
      'Color', col_txt, 'FontSize', 12, 'FontWeight', 'bold');
applyDarkAxes(ax4, col_grid, col_txt);

%% ---- Super-title -----------------------------------------------
annotation('textbox', [0 0.97 1 0.03], ...
    'String', sprintf( ...
    ['Front Wing Clearance — CG pivot  |  a = %.0f mm  |  b = %.0f mm  |  ' ...
     'x_{ref} = %.2f mm  |  z_{ref} = %.0f mm  |  ' ...
     '\\theta_{study} = %.1f°  |  \\Deltaz = %.4f mm  |  \\theta_{crit} = %.4f°'], ...
    a, b, x_ref, z_ref, theta_study_deg, z_tip_study, theta_crit), ...
    'Color', [0.65 0.65 0.65], 'FontSize', 9.5, ...
    'HorizontalAlignment', 'center', 'EdgeColor', 'none', 'BackgroundColor', 'none');

set(fig, 'Units', 'normalized');
fprintf('Plot rendered successfully.\n');

%% ============================================================
%  LOCAL FUNCTIONS
%% ============================================================

function [clearance, drop_pitch, z_fa] = wingTipClearance( ...
        theta_deg, heave_cg_mm, a_mm, x_ref, z_ref)
% WINGTIPCLEARANCE  Front wing ground clearance — pivot at CG.
%
%   theta_deg   - pitch angle (deg), +ve = nose down
%   heave_cg_mm - vertical displacement of CG (mm), +ve = upward
%   a_mm        - distance CG to front axle (mm)  [car.a * 1000]
%   x_ref       - reference point fwd of front axle (mm)
%   z_ref       - reference point static height above ground (mm)
%
%   clearance   - ground clearance at wing tip (mm); <0 means contact
%   drop_pitch  - total tip drop due to pitch (mm)
%   z_fa        - front axle height above ground (mm)

    th_rad     = theta_deg * pi / 180;
    z_fa       = heave_cg_mm - a_mm  * sin(th_rad);
    z_tip      = z_fa        + z_ref * cos(th_rad) - x_ref * sin(th_rad);
    drop_pitch = (a_mm + x_ref) * sin(th_rad) - z_ref * (1 - cos(th_rad));
    clearance  = z_tip;
end

function applyDarkAxes(ax, col_grid, col_txt)
    ax.Color          = [0.08 0.08 0.12];
    ax.XColor         = col_txt;
    ax.YColor         = col_txt;
    ax.GridColor      = col_grid;
    ax.MinorGridColor = col_grid;
    ax.GridAlpha      = 0.45;
    ax.MinorGridAlpha = 0.25;
    ax.Box            = 'on';
    ax.XMinorGrid     = 'on';
    ax.YMinorGrid     = 'on';
    grid(ax, 'on');
    hold(ax, 'off');
end

function cmap = customClearanceColormap(n)
    n1   = round(n * 0.35);
    n2   = round(n * 0.20);
    n3   = n - n1 - n2;
    r1   = [linspace(0.70,0.85,n1)', linspace(0.10,0.55,n1)', linspace(0.10,0.05,n1)'];
    r2   = [linspace(0.85,0.95,n2)', linspace(0.55,0.72,n2)', linspace(0.05,0.05,n2)'];
    r3   = [linspace(0.05,0.05,n3)', linspace(0.45,0.70,n3)', linspace(0.35,0.55,n3)'];
    cmap = [r1; r2; r3];
end