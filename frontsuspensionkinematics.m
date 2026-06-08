%% =========================================================================
%  Suspension Kinematics of Double Wishbone Rack and Pinion for Front Axle
%  =========================================================================
clear; clc; close all
paramR26;

%% =========================================================================
%  >>>  USER PARAMETERS — EDIT HERE  <
%  =========================================================================

par.umx          = 0.08255 / 2;
par.phmx_droop   = 2.12  / 180 * pi;
par.phmx_jounce  = 5.43  / 180 * pi;

par.rvwk = [0.89604;  -0.605;      0.2032    ];
par.rvak = [0.834;    -0.201;      0.104816  ];
par.rvbk = [1.04;     -0.201;      0.104816  ];
par.rvck = [0.90186;  -0.5583;     0.121266  ];
par.rvdk = [0.8316;   -0.2819;     0.2420437 ];
par.rvek = [1.002;    -0.2819;     0.2420437 ];
par.rvfk = [0.88988;  -0.5341618;  0.2926    ];
par.rvrk = [0.8404;   -0.21368;    0.164     ];
par.rvqk = [0.80924;  -0.49872;    0.1982    ];

toe0  = 0 / 180 * pi;
camb0 = 0 / 180 * pi;
rs    = car.r_wheel;

wheelbase           = car.wheelbase;
c_factor_in_per_rev = 4.71;

n_droop  = 10;
n_jounce = 15;
m        = 21;

u = linspace(-par.umx, par.umx, m);

wheel_side = 'Right';
conv_str   = 'ISO 8855';

%% =========================================================================
%  END OF USER PARAMETERS
%% =========================================================================

en0 = [0; 0; 1];

eyrk = [toe0; 1; -camb0];  eyrk = eyrk / norm(eyrk);
exk  = cross(eyrk, en0);   exk  = exk  / norm(exk);
eyk  = cross(en0,  exk);
ezk  = cross(exk,  eyrk);
rwpk = -rs * ezk;

rcfk = par.rvfk - par.rvck;
ecfk = rcfk / norm(rcfk);
si   = atan2(-ecfk(2), ecfk(3));
nu   = atan2(-ecfk(1), ecfk(3));
disp(['sigma = ', num2str(si * 180/pi), ' deg'])
disp(['nue   = ', num2str(nu * 180/pi), ' deg'])

rcpk    = par.rvwk + rwpk - par.rvck;
rsck    = -(en0.' * rcpk) / (en0.' * ecfk) * ecfk;
co      = -exk' * (rsck + rcpk);
sr_geom =  eyk' * (rsck + rcpk);
disp(['caster offset = ', num2str(co)])
disp(['scrub radius  = ', num2str(sr_geom)])

sign_block = {
    ['Analyzed Wheel: Front-', wheel_side, ' (negative Y)'], ...
    ['Opposite Wheel: Front-', conditional_string(wheel_side)], ...
    ['Coord. system: ', conv_str, ' (X fwd, Y left, Z up)'], ...
    'Toe  (+): toe-in  (+Z rotation)', ...
    'Camber (+): top outboard  (+X rotation)', ...
    'Phi  (+): jounce,  Phi (-): droop', ...
    'u    (+): rack displ. positive => RIGHT steer  |  u (-): negative => LEFT steer', ...
    'SW angle (+): LEFT turn'
};

n = n_droop + n_jounce - 1;

phi_droop  = linspace(-par.phmx_droop,  0, n_droop);
phi_jounce = linspace(0, par.phmx_jounce, n_jounce);
phi        = [phi_droop(1:end-1), phi_jounce];

[~, n0] = min(abs(phi));

disp('Suspension travel discretization:')
disp(['  Droop:  ', num2str(n_droop),  ' steps, ', num2str(-par.phmx_droop  * 180/pi), ' deg'])
disp(['  Jounce: ', num2str(n_jounce), ' steps, +', num2str( par.phmx_jounce * 180/pi), ' deg'])
disp(['  Total:  ', num2str(n), ' steps'])
disp(['  Design position at index n0 = ', num2str(n0), ' (phi = ', num2str(phi(n0)*180/pi, '%.3f'), ' deg)'])

[avw_des, ~, ~, ~] = fun_05_dblwb_kin(0, 0, par);
eyrv_des       = avw_des * eyrk;
eyrv_des_xy    = eyrv_des(1:2) / norm(eyrv_des(1:2));
toe_design_ref = atan2(-eyrv_des_xy(1), eyrv_des_xy(2));

%% -----------------------------------------------------------------------
%  MAIN KINEMATICS LOOP
%% -----------------------------------------------------------------------
xw = zeros(n,m); yw = xw; zw = xw;
xp = xw;         yp = xw; zp = xw;
del_analyzed = xw; toe_analyzed = xw; camb_analyzed = xw; ddel = xw;

for i = 1:n
    for j = 1:m
        [avw, rvwv, del_analyzed(i,j), pd] = fun_05_dblwb_kin(phi(i), u(j), par);
        eyrv  = avw * eyrk;
        rvpv  = rvwv + avw * rwpk;
        xw(i,j) = rvwv(1);  yw(i,j) = rvwv(2);  zw(i,j) = rvwv(3);
        xp(i,j) = rvpv(1);  yp(i,j) = rvpv(2);  zp(i,j) = rvpv(3);

        eyrv_xy = eyrv(1:2) / norm(eyrv(1:2));
        toe_analyzed(i,j)  = atan2(-eyrv_xy(1), eyrv_xy(2)) - toe_design_ref;
        camb_analyzed(i,j) = atan2(-eyrv(3), eyrv(2));
        ddel(i,j) = norm(pd(:,2));
    end
end

[~, n0] = min(abs(phi));
[~, m0] = min(abs(u));
rvpk    = par.rvwk + rwpk;

%% -----------------------------------------------------------------------
%  STEER ANGLES FOR BOTH WHEELS
%% -----------------------------------------------------------------------
del_right = zeros(1, m);
del_left  = zeros(1, m);

if strcmp(wheel_side, 'Right')
    del_right = del_analyzed(n0, :);
    par_opposite = par;
    fields = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk','rvrk','rvqk'};
    for f = 1:length(fields)
        par_opposite.(fields{f})(2) = -par.(fields{f})(2);
    end
    for j = 1:m
        [~, ~, del_left(j), ~] = fun_05_dblwb_kin(phi(n0), u(j), par_opposite);
    end
else
    error('Script assumes wheel_side = ''Right''.');
end

u_inch       = u * 39.3701;
sw_angle_deg = -(u_inch / c_factor_in_per_rev) * 360;

%% -----------------------------------------------------------------------
%  NEUTRAL POSITION DIAGNOSTIC
%% -----------------------------------------------------------------------
[~, ~, delta_R_neut, ~] = fun_05_dblwb_kin(0, 0, par);
par_opposite = par;
fields = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk','rvrk','rvqk'};
for f = 1:length(fields)
    par_opposite.(fields{f})(2) = -par.(fields{f})(2);
end
[~, ~, delta_L_neut, ~] = fun_05_dblwb_kin(0, 0, par_opposite);

eyrk_right = [toe0; 1; -camb0];  eyrk_right = eyrk_right / norm(eyrk_right);
[avw_right, ~, ~, ~] = fun_05_dblwb_kin(0, 0, par);
eyrv_right = avw_right * eyrk_right;
camber_R_neut = atan2(eyrv_right(3), eyrv_right(2)) * 180/pi;

[avw_left, ~, ~, ~] = fun_05_dblwb_kin(0, 0, par_opposite);
eyrv_left = avw_left * eyrk_right;
camber_L_neut = atan2(eyrv_left(3), eyrv_left(2)) * 180/pi;

fprintf('\n===== NEUTRAL POSITION ANALYSIS (exact u=0, phi=0) =====\n')
fprintf('Steer Angle - Left:   %.3f deg\n', delta_L_neut * 180/pi)
fprintf('Steer Angle - Right:  %.3f deg\n', delta_R_neut * 180/pi)
fprintf('Camber Angle - Left:  %.3f deg\n', camber_L_neut)
fprintf('Camber Angle - Right: %.3f deg\n', camber_R_neut)

if abs((delta_L_neut - delta_R_neut) * 180/pi) > 0.5
    fprintf('WARNING: Large steer asymmetry: %.3f deg\n', abs((delta_L_neut - delta_R_neut)*180/pi))
end
if abs(camber_L_neut - camber_R_neut) > 0.3
    fprintf('WARNING: Large camber asymmetry: %.3f deg\n', abs(camber_L_neut - camber_R_neut))
end

%% -----------------------------------------------------------------------
%  HELPER FUNCTIONS
%% -----------------------------------------------------------------------
function opposite = conditional_string(wheel_side)
    if strcmp(wheel_side, 'Right'), opposite = 'Left';
    else, opposite = 'Right'; end
end

function add_sign_box(sign_block, wheel_side)
    annotation('textbox', [0.00, 0.00, 1.00, 0.10], ...
        'String', sign_block, 'FontSize', 7.5, 'FontName', 'Courier New', ...
        'BackgroundColor', [0.95 0.95 0.85], 'EdgeColor', [0.40 0.40 0.20], ...
        'FitBoxToText', 'off', 'LineWidth', 1, 'Interpreter', 'none', ...
        'VerticalAlignment', 'middle');
end

function s = ternary_str(condition, true_str, false_str)
    if condition, s = true_str; else, s = false_str; end
end

%% -----------------------------------------------------------------------
%  FIGURE 1: Wheel Centre Paths
%% -----------------------------------------------------------------------
figure('Name', ['Figure 1 - Wheel Center Paths  |  Analyzed: ', wheel_side, ' Wheel'])

axes('position',[0.05, 0.12, 0.18, 0.78])
hold on, axis equal, grid on
title({'Longitudinal Plane (XZ)'; ['Front-', wheel_side]}, 'FontWeight','bold')
xlabel('x  [m]  (+ forward)'), ylabel('z  [m]  (+ upward)')
h1 = plot(xw(:,m0), zw(:,m0), 'b-',  'LineWidth',1.5);
h2 = plot(xp(:,m0), zp(:,m0), 'b--', 'LineWidth',1.5);
h3 = plot(par.rvwk(1), par.rvwk(3), 'ok', 'MarkerFaceColor','k', 'MarkerSize',6);
h4 = plot(rvpk(1), rvpk(3), 'o', 'Color','cyan', 'MarkerFaceColor','cyan', 'MarkerSize',6);
legend([h1,h2,h3,h4], 'W center','Contact P','W design','P design', 'Location','best','FontSize',8)

axes('position',[0.28, 0.12, 0.18, 0.78])
hold on, axis equal, grid on
title({'Lateral Plane (YZ)'; ['Front-', wheel_side]}, 'FontWeight','bold')
xlabel({'y  [m]'; '(+left, -Y=right)'}), ylabel('z  [m]  (+ upward)')
h1 = plot(yw(:,m0), zw(:,m0), 'r-',  'LineWidth',1.5);
h2 = plot(yp(:,m0), zp(:,m0), 'r--', 'LineWidth',1.5);
h3 = plot(par.rvwk(2), par.rvwk(3), 'ok', 'MarkerFaceColor','k', 'MarkerSize',6);
h4 = plot(rvpk(2), rvpk(3), 'o', 'Color','cyan', 'MarkerFaceColor','cyan', 'MarkerSize',6);
legend([h1,h2,h3,h4], 'W center','Contact P','W design','P design', 'Location','best','FontSize',8)

axes('position',[0.55, 0.55, 0.42, 0.35])
surf(u*1000, phi*180/pi, toe_analyzed*180/pi), colormap('parula')
grid on, view(-40,10)
title({'Toe Angle Map'; '(+) = toe-in'}, 'FontWeight','bold')
xlabel('u  [mm]  (+: right steer)'), ylabel('\phi [deg] (+:jounce/-:droop)'), zlabel('Toe [deg]')

axes('position',[0.55, 0.12, 0.42, 0.35])
surf(u*1000, phi*180/pi, camb_analyzed*180/pi), colormap('parula')
grid on, view(-40,10)
title({'Camber Angle Map'; '(+) = top outboard'}, 'FontWeight','bold')
xlabel('u  [mm]  (+: right steer)'), ylabel('\phi [deg] (+:jounce/-:droop)'), zlabel('Camber [deg]')

add_sign_box(sign_block, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 2: Steering Kinematics
%% -----------------------------------------------------------------------
figure('Name', ['Figure 2 - Steering Kinematics  |  Analyzed: ', wheel_side, ' Wheel'])

d1  = del_right;
d2  = del_left;
a   = wheelbase;
s   = 2 * abs(par.rvwk(2));
d2a = atan2(a * tan(d1), a - s * tan(d1));

axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Steering Angles vs Rack Displacement'; 'Front Axle  |  Design ride height'}, 'FontWeight','bold')
xlabel('Rack disp.  u  [mm]  (+: right steer)'), ylabel('Steer angle  \delta  [deg]  (+: left steer)')
plot(u*1000, d1*180/pi, 'b-',  'LineWidth',1.5, 'DisplayName','Right wheel  \delta_{right}')
plot(u*1000, d2*180/pi, 'r--', 'LineWidth',1.5, 'DisplayName','Left wheel  \delta_{left}')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')

axes('position',[0.06, 0.14, 0.38, 0.33])
hold on, grid on
title({'Steering Sensitivity  d\delta/du'; '(+u): right steer  |  Design height'}, 'FontWeight','bold')
xlabel('Rack disp.  u  [mm]  (+: right steer)'), ylabel('d\delta / du  [rad/m]')
plot(u*1000, ddel(n0,:), 'm-', 'LineWidth',1.5, 'DisplayName','d\delta/du (analyzed wheel)')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')

axes('position',[0.54, 0.14, 0.43, 0.77])
hold on, axis equal, grid on
title({'Ackermann Comparison'; '\delta_{outer} vs \delta_{inner} (both wheels)'}, 'FontWeight','bold')
xlabel('\delta_{outer}  [deg]  (+: left steer)'), ylabel('\delta_{inner}  [deg]  (+: left steer)')
plot(d1*180/pi, d2*180/pi,  'b-',  'LineWidth',2.0, 'DisplayName','Kinematic (right vs left)')
plot(d1*180/pi, d2a*180/pi, 'r--', 'LineWidth',1.5, 'DisplayName', ['Ideal Ackermann (a=', num2str(a), 'm)'])
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')

add_sign_box(sign_block, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 3: Heave Response
%% -----------------------------------------------------------------------
figure('Name', ['Figure 3 - Heave Response  |  Analyzed: ', wheel_side, ' Wheel'])

z_design   = zw(n0, m0);
heave      = (zw(:, m0) - z_design) * 1000;
toe_heave  = (toe_analyzed(:, m0)  - toe_analyzed(n0,  m0)) * 180/pi;
camb_heave = (camb_analyzed(:, m0) - camb_analyzed(n0, m0)) * 180/pi;

jounce_idx = find(heave >= 0);
droop_idx  = find(heave <= 0);
hj = heave(jounce_idx);  tj = toe_heave(jounce_idx);
hd = heave(droop_idx);   td = toe_heave(droop_idx);

axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({['Toe Change vs Heave  |  Front-', wheel_side, ' Wheel']; 'Neutral steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)'), ylabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
fill([hj; hj(end); hj(1)], [tj; 0; 0], [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd; hd(end); hd(1)], [td; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave, toe_heave, 'b-o', 'LineWidth',2, 'MarkerSize',5, 'DisplayName','Toe change')
plot(heave(n0), toe_heave(n0), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
text(max(heave)*0.1, max(abs(toe_heave))*0.75, 'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(heave)*0.9, max(abs(toe_heave))*0.75, 'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
legend('Location','southwest','FontSize',9)

% Ride camber rate [deg/m]
heave_m      = (zw(:, m0) - z_design);          % heave in metres
ride_camber_rate = gradient(camb_heave, heave_m * 1000) / 1000 * 1000;
% deg/mm * 1000 = deg/m — use polyfit over full range for a clean rate
p_camb_heave = polyfit(heave_m, camb_heave, 1);  % linear fit
ride_camber_rate_avg = p_camb_heave(1);           % [deg/m]  slope

axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Front-', wheel_side, ' Wheel']; ...
       ['Neutral steering  (u = 0)  |  Ride camber rate: ', ...
        sprintf('%.2f', ride_camber_rate_avg), ' deg/m']}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
cj      = camb_heave(jounce_idx);
cd_vals = camb_heave(droop_idx);
fill([hj; hj(end); hj(1)], [cj; 0; 0],      [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd; hd(end); hd(1)], [cd_vals; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave, camb_heave, 'r-s', 'LineWidth',2, 'MarkerSize',5, 'DisplayName','Camber change')
plot(heave(n0), camb_heave(n0), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
% Plot linear fit line
heave_fit_mm = linspace(min(heave), max(heave), 100);
camb_fit     = polyval(p_camb_heave, heave_fit_mm/1000);
plot(heave_fit_mm, camb_fit, 'k--', 'LineWidth', 1, 'DisplayName', ...
    sprintf('Linear fit: %.2f deg/m', ride_camber_rate_avg))
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
text(max(heave)*0.4, max(abs(camb_heave))*0.75, 'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(heave)*0.5, max(abs(camb_heave))*0.75, 'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
legend('Location','southwest','FontSize',9)

axes('position',[0.20, 0.14, 0.55, 0.33])
hold on, grid on
title({'Toe-Camber Correlation During Heave'; ['Front-', wheel_side, ' Wheel  |  Coloured by heave [mm]']}, 'FontWeight','bold')
xlabel('\Delta\delta_{toe}  [deg]  (+: toe-in)'), ylabel('\Delta\gamma  [deg]  (+: top outboard)')
scatter(toe_heave, camb_heave, 80, heave, 'filled','o','DisplayName','Heave steps')
cbar = colorbar;
cbar.Label.String = 'Heave  h  [mm]  (+: jounce)';  cbar.Label.FontSize = 9;
plot(toe_heave(n0), camb_heave(n0), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')

add_sign_box(sign_block, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 4: Camber Change vs Body Roll (0 to 1 deg) — NEW
%% -----------------------------------------------------------------------
figure('Name', 'Figure 4 - Camber vs Body Roll  |  Front Axle  |  No Steering')

% Mirror left wheel hardpoints
par_roll_l = par;
fields_roll = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk','rvrk','rvqk'};
for f = 1:length(fields_roll)
    par_roll_l.(fields_roll{f})(2) = -par.(fields_roll{f})(2);
end

% Design camber references
[avw_dr, ~, ~, ~] = fun_05_dblwb_kin(0, 0, par);
eyrv_ref_r = avw_dr * eyrk;
camb_ref_r = atan2(eyrv_ref_r(3), eyrv_ref_r(2)) * 180/pi;

[avw_dl, ~, ~, ~] = fun_05_dblwb_kin(0, 0, par_roll_l);
eyrv_ref_l = avw_dl * eyrk;
camb_ref_l = atan2(eyrv_ref_l(3), eyrv_ref_l(2)) * 180/pi;

% Sweep body roll 0 to 1 deg
n_roll          = 100;
roll_sweep_deg  = linspace(0, 1, n_roll);
camb_outer_roll = zeros(1, n_roll);
camb_inner_roll = zeros(1, n_roll);

for k = 1:n_roll
    % Each wheel sees half the total roll as phi
    phi_o =  (roll_sweep_deg(k) / 2) / 180 * pi;
    phi_i = -(roll_sweep_deg(k) / 2) / 180 * pi;

    [avw_o, ~, ~, ~] = fun_05_dblwb_kin(phi_o, 0, par);
    eyrv_o = avw_o * eyrk;
    camb_outer_roll(k) = atan2(eyrv_o(3), eyrv_o(2)) * 180/pi - camb_ref_r;

    [avw_i, ~, ~, ~] = fun_05_dblwb_kin(phi_i, 0, par_roll_l);
    eyrv_i = avw_i * eyrk;
    camb_inner_roll(k) = atan2(eyrv_i(3), eyrv_i(2)) * 180/pi - camb_ref_l;
end

% Road-relative camber (includes body roll contribution)
camb_outer_road_roll = camb_outer_roll - roll_sweep_deg / 2;
camb_inner_road_roll = camb_inner_roll + roll_sweep_deg / 2;

% Roll camber rates [deg/deg]
p_outer_susp = polyfit(roll_sweep_deg, camb_outer_roll,      1);
p_inner_susp = polyfit(roll_sweep_deg, camb_inner_roll,      1);
p_outer_road = polyfit(roll_sweep_deg, camb_outer_road_roll, 1);
p_inner_road = polyfit(roll_sweep_deg, camb_inner_road_roll, 1);

rate_outer_susp = p_outer_susp(1);   % [deg/deg]
rate_inner_susp = p_inner_susp(1);
rate_outer_road = p_outer_road(1);
rate_inner_road = p_inner_road(1);

% Left: Suspension camber change vs body roll
axes('position', [0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Suspension Camber Change vs Body Roll'; ...
       ['Outer: ', sprintf('%.4f', rate_outer_susp), ' deg/deg  |  ', ...
        'Inner: ', sprintf('%.4f', rate_inner_susp), ' deg/deg']}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]  (right corner = positive)')
ylabel('\Delta\gamma_{susp}  [deg/deg body roll]  (+: top outboard)')
plot(roll_sweep_deg, camb_outer_roll, 'b-',  'LineWidth', 2, 'DisplayName', ...
    ['Outer (right, jounce)  ', sprintf('%.4f deg/deg', rate_outer_susp)])
plot(roll_sweep_deg, camb_inner_roll, 'r--', 'LineWidth', 2, 'DisplayName', ...
    ['Inner (left, droop)    ', sprintf('%.4f deg/deg', rate_inner_susp)])
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

% Right: Road-relative camber vs body roll
axes('position', [0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({'Road-Relative Camber vs Body Roll'; ...
       ['Outer: ', sprintf('%.4f', rate_outer_road), ' deg/deg  |  ', ...
        'Inner: ', sprintf('%.4f', rate_inner_road), ' deg/deg']}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]  (right corner = positive)')
ylabel('\Delta\gamma_{road}  [deg/deg body roll]  (+: top outboard)')
plot(roll_sweep_deg, camb_outer_road_roll, 'b-',  'LineWidth', 2, 'DisplayName', ...
    ['Outer (right)  ', sprintf('%.4f deg/deg', rate_outer_road)])
plot(roll_sweep_deg, camb_inner_road_roll, 'r--', 'LineWidth', 2, 'DisplayName', ...
    ['Inner (left)   ', sprintf('%.4f deg/deg', rate_inner_road)])
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

% Bottom: Comparison at 1 deg roll
axes('position', [0.20, 0.14, 0.55, 0.33])
hold on, grid on
title({'Suspension vs Road-Relative Camber  |  0 to 1° Body Roll'; ...
       ['No steering  |  Road-relative rates:  Outer = ', ...
        sprintf('%.4f', rate_outer_road), ' deg/deg  |  Inner = ', ...
        sprintf('%.4f', rate_inner_road), ' deg/deg']}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
plot(roll_sweep_deg, camb_outer_roll,      'b-',  'LineWidth', 2,   'DisplayName', 'Outer suspension \Delta\gamma')
plot(roll_sweep_deg, camb_outer_road_roll, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Outer road-relative \Delta\gamma')
plot(roll_sweep_deg, camb_inner_roll,      'r-',  'LineWidth', 2,   'DisplayName', 'Inner suspension \Delta\gamma')
plot(roll_sweep_deg, camb_inner_road_roll, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Inner road-relative \Delta\gamma')
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
text(1.01, camb_outer_road_roll(end), sprintf('%+.3f° (%+.4f deg/deg)', camb_outer_road_roll(end), rate_outer_road), ...
    'FontSize', 9, 'Color', 'b', 'FontWeight', 'bold')
text(1.01, camb_inner_road_roll(end), sprintf('%+.3f° (%+.4f deg/deg)', camb_inner_road_roll(end), rate_inner_road), ...
    'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold')
legend('Location', 'best', 'FontSize', 9)

roll_sign_block = {
    ['Analyzed Wheel: Front-', wheel_side, '  |  No steering input  (u = 0)'], ...
    'Body roll split 50/50 — each wheel sees half total roll as phi', ...
    'Suspension camber: change due to suspension geometry only', ...
    'Road-relative camber: suspension change + body roll tilt contribution'
};
add_sign_box(roll_sign_block, wheel_side)


%% -----------------------------------------------------------------------
%  FIGURE 5: Steering Response  (was Figure 4)
%% -----------------------------------------------------------------------
figure('Name', ['Figure 5 - Steering Response  |  Analyzed: ', wheel_side, ' Wheel'])

steer_angle  = (del_analyzed(n0, :) - del_analyzed(n0, m0)) * 180/pi;
camb_steer   = (camb_analyzed(n0, :) - camb_analyzed(n0, m0)) * 180/pi;

u_col           = u(:);
steer_angle_col = steer_angle(:);
camb_steer_col  = camb_steer(:);

right_idx = find(u_col >= 0);
left_idx  = find(u_col <= 0);
ur = u_col(right_idx) * 1000;
ul = u_col(left_idx)  * 1000;
[~, u0_idx] = min(abs(u));

axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({['Steer Angle vs Rack Displacement  |  Front-', wheel_side, ' Wheel']; 'Design ride height  (\phi = 0)'}, 'FontWeight','bold')
xlabel('Rack disp.  u  [mm]  (+: right steer)'), ylabel('\delta  [deg]  (+: left steer)')
sr = steer_angle_col(right_idx);  sl = steer_angle_col(left_idx);
fill([ur; ur(end); ur(1)], [sr; 0; 0], [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([ul; ul(end); ul(1)], [sl; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(u*1000, steer_angle, 'b-o', 'LineWidth',2, 'MarkerSize',5, 'DisplayName','Steer angle  \delta')
plot(u(u0_idx)*1000, steer_angle(u0_idx), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Neutral (u=0)')
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
text(max(u)*1000*0.1, max(abs(steer_angle))*0.75, 'STEER RIGHT', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(u)*1000*0.9, max(abs(steer_angle))*0.75, 'STEER LEFT',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
legend('Location','southwest','FontSize',9)

axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Steering  |  Front-', wheel_side, ' Wheel']; 'Design ride height  (\phi = 0)'}, 'FontWeight','bold')
xlabel('Rack disp.  u  [mm]  (+: right steer)'), ylabel('\Delta\gamma  [deg]  (+: top outboard)')
cr = camb_steer_col(right_idx);  cl = camb_steer_col(left_idx);
fill([ur; ur(end); ur(1)], [cr; 0; 0], [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([ul; ul(end); ul(1)], [cl; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(u*1000, camb_steer, 'r-s', 'LineWidth',2, 'MarkerSize',5, 'DisplayName','Camber change')
plot(u(u0_idx)*1000, camb_steer(u0_idx), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Neutral (u=0)')
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
text(max(u)*1000*0.1, max(abs(camb_steer))*0.75, 'STEER RIGHT', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(u)*1000*0.9, max(abs(camb_steer))*0.75, 'STEER LEFT',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
legend('Location','northwest','FontSize',9)

axes('position',[0.20, 0.14, 0.55, 0.33])
hold on, grid on
title({'Steer-Camber Correlation During Steering'; ['Front-', wheel_side, ' Wheel  |  Coloured by rack disp. [mm]']}, 'FontWeight','bold')
xlabel('\delta  [deg]  (+: left steer)'), ylabel('\Delta\gamma  [deg]  (+: top outboard)')
scatter(steer_angle, camb_steer, 80, u*1000, 'filled','o','DisplayName','Steering steps')
cbar = colorbar;
cbar.Label.String = 'Rack disp.  u  [mm]  (+: right steer)';  cbar.Label.FontSize = 9;
plot(steer_angle(u0_idx), camb_steer(u0_idx), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Neutral (u=0)')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')

add_sign_box(sign_block, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 6: Steering Ratio & Ackermann  (was Figure 5)
%% -----------------------------------------------------------------------
figure('Name', ['Figure 6 - Steering Ratio & Ackermann  |  Analyzed: ', wheel_side, ' Wheel'])

d1_deg = del_right * 180/pi;
d2_deg = del_left  * 180/pi;

valid = abs(d1_deg) > 0.1;
d1_safe = d1_deg;  d2_safe = d2_deg;
d1_safe(~valid) = NaN;  d2_safe(~valid) = NaN;

SR_right = sw_angle_deg ./ d1_safe;
SR_left  = sw_angle_deg ./ d2_safe;

a = wheelbase;
s = 2 * abs(par.rvwk(2));

fprintf('\n===== ACKERMANN PARAMETERS =====\n')
fprintf('Wheelbase  a: %.4f m = %.1f mm\n', a, a*1000)
fprintf('Track width s: %.4f m = %.1f mm\n', s, s*1000)
fprintf('s/a ratio: %.4f\n\n', s/a)

pct_ackermann = zeros(1, m);
d2a_signed    = zeros(1, m);
valid_ack     = abs(d1_deg) > 0.1;
min_steer_threshold = 5.0;

for k = 1:m
    if abs(d1_deg(k)) <= abs(d2_deg(k))
        d_outer_abs = abs(d1_deg(k));  d_inner_actual_abs = abs(d2_deg(k));  sign_outer = sign(d1_deg(k));
    else
        d_outer_abs = abs(d2_deg(k));  d_inner_actual_abs = abs(d1_deg(k));  sign_outer = sign(d2_deg(k));
    end
    if d_outer_abs < min_steer_threshold || d_outer_abs < 0.05
        pct_ackermann(k) = NaN;  d2a_signed(k) = NaN;  continue
    end
    cot_outer       = 1 / tan(d_outer_abs * pi/180);
    cot_ideal_inner = cot_outer - s/a;
    if cot_ideal_inner <= 0
        pct_ackermann(k) = NaN;  d2a_signed(k) = NaN;  continue
    end
    ideal_inner_abs = atan(1 / cot_ideal_inner) * 180/pi;
    d2a_signed(k)   = sign_outer * ideal_inner_abs;
    actual_spread   = d_inner_actual_abs - d_outer_abs;
    ideal_spread    = ideal_inner_abs    - d_outer_abs;
    if ideal_spread > 0.001
        pct_ackermann(k) = (actual_spread / ideal_spread) * 100;
    else
        pct_ackermann(k) = NaN;
    end
end

opposite_side = 'Left';
sr_block = {
    ['Analyzed: Front-', wheel_side, '  |  Opposite: Front-', opposite_side, '  |  C-factor = ', num2str(c_factor_in_per_rev), ' in/rev'], ...
    ['Track width: s = ', num2str(s*1000,'%.1f'), ' mm  |  Wheelbase: a = ', num2str(a*1000,'%.0f'), ' mm'], ...
    'Steering Ratio = SW angle / road wheel angle  (+ SW: left steer)', ...
    '% Ackermann: 0% = parallel steer,  100% = ideal Ackermann,  >100% = over-Ackermann'
};

axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Steering Ratio vs Steering Wheel Angle'; ['Front Axle  |  Design ride height  (\phi = 0)']}, 'FontWeight','bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)'), ylabel('Steering ratio  SR  [-]')
plot(sw_angle_deg(valid), SR_right(valid), 'b-o', 'LineWidth',2, 'MarkerSize',5, 'DisplayName', ['SR Right wheel (', wheel_side, ')'])
plot(sw_angle_deg(valid), SR_left(valid),  'r--s','LineWidth',2, 'MarkerSize',5, 'DisplayName',['SR ', opposite_side, ' wheel'])
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
% Through-centre SR — interpolate from raw data at SW=0
[~, idx_centre] = min(abs(sw_angle_deg));
sr_centre_R = SR_right(idx_centre);
sr_centre_L = SR_left(idx_centre);

% Mark through-centre on plot
plot(0, sr_centre_R, 'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'b', ...
    'DisplayName', sprintf('SR_{right} @ centre = %.3f', sr_centre_R))
plot(0, sr_centre_L, 'rv', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('SR_{left}  @ centre = %.3f', sr_centre_L))

% Annotate values
text(2, sr_centre_R + 0.15, sprintf('SR_{right} = %.3f', sr_centre_R), ...
    'FontSize', 9, 'Color', 'b', 'FontWeight', 'bold')
text(2, sr_centre_L - 0.15, sprintf('SR_{left}  = %.3f', sr_centre_L), ...
    'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold', ...
    'VerticalAlignment', 'top')

fprintf('\n===== STEERING RATIO THROUGH CENTRE =====\n')
fprintf('SR through centre - Right: %.4f\n', sr_centre_R)
fprintf('SR through centre - Left:  %.4f\n', sr_centre_L)
fprintf('SR through centre - Mean:  %.4f\n', mean([sr_centre_R, sr_centre_L]))
legend('Location','best','FontSize',9)

axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({'Road Wheel Steer Angles vs Steering Wheel Angle'; ['Front Axle  |  Design ride height  (\phi = 0)']}, 'FontWeight','bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)'), ylabel('Road wheel angle  \delta  [deg]  (+: left steer)')
plot(sw_angle_deg, d1_deg,     'b-',  'LineWidth', 2.0, 'DisplayName', ['Right wheel  \delta_{right} (', wheel_side, ')'])
plot(sw_angle_deg, d2_deg,     'r--', 'LineWidth', 2.0, 'DisplayName', ['Left wheel  \delta_{left} (', opposite_side, ')'])
plot(sw_angle_deg, d2a_signed, 'k:',  'LineWidth', 1.5, 'DisplayName', 'Ideal inner wheel  \delta_{inner,ack}')
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
legend('Location','best','FontSize',9)

axes('position',[0.06, 0.14, 0.38, 0.33])
hold on, grid on
title({'% Ackermann vs Steering Wheel Angle'; '100% = ideal Ackermann  |  0% = parallel steer'}, 'FontWeight','bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)'), ylabel('% Ackermann  [%]')
plot(sw_angle_deg(valid_ack), pct_ackermann(valid_ack), 'm-o', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', '% Ackermann')
yline(100, 'k--', 'LineWidth', 1.2, 'DisplayName', '100% (ideal Ackermann)')
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
ylims = ylim;
annotation('rectangle', [0.06 + (0.68-0.06)*(100/ylims(2)), 0.68, 0.38*(1-100/ylims(2)), ylims(2)-100], ...
    'FaceColor', [0.9 1.0 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3)
legend('Location','best','FontSize',9)

axes('position',[0.56, 0.14, 0.38, 0.33])
axis off
title({'Summary Table  |  Key Positions'; ['C-factor: ', num2str(c_factor_in_per_rev), ' in/rev']}, 'FontWeight','bold')
n_rows = 9;
key_idx = round(linspace(1, m, n_rows));
col_sw  = sw_angle_deg(key_idx);
col_d1  = d1_deg(key_idx);
col_d2  = d2_deg(key_idx);
col_sr1 = SR_right(key_idx);
col_sr2 = SR_left(key_idx);
col_ack = pct_ackermann(key_idx);
headers = {'SW [deg]', '\delta_R [deg]', '\delta_L [deg]', 'SR_R', 'SR_L', '% Ack'};
col_x   = [0.05, 0.20, 0.35, 0.50, 0.65, 0.80];
row_y   = 0.92;
row_dy  = 0.095;
for c = 1:length(headers)
    text(col_x(c), row_y, headers{c}, 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Interpreter', 'tex')
end
annotation('line', [0.56, 0.94], [0.54, 0.54], 'Color','k','LineWidth',1.5)
for r = 1:length(key_idx)
    ry = row_y - r * row_dy;
    vals = {sprintf('%.1f', col_sw(r)), sprintf('%.2f', col_d1(r)), sprintf('%.2f', col_d2(r)), ...
            sprintf('%.1f', col_sr1(r)), sprintf('%.1f', col_sr2(r)), sprintf('%.1f', col_ack(r))};
    if mod(r,2) == 0
        annotation('rectangle', [0.56, ry-0.04, 0.38, row_dy], 'FaceColor',[0.93 0.93 0.93],'EdgeColor','none','FaceAlpha',0.6)
    end
    for c = 1:length(vals)
        text(col_x(c), ry, vals{c}, 'FontSize', 8, 'HorizontalAlignment', 'center', 'Interpreter', 'none')
    end
end

add_sign_box(sr_block, wheel_side)

% Static Ackermann at key steering angles
fprintf('\n===== STATIC ACKERMANN SUMMARY =====\n')
key_sw_ack = [30, 45, 60, 90, 120];
key_sw_ack = key_sw_ack(key_sw_ack <= max(abs(sw_angle_deg)));
fprintf('%12s  %15s\n', 'SW angle [deg]', '% Ackermann')
for k = 1:length(key_sw_ack)
    % Interpolate pct_ackermann at this SW angle
    % Use positive SW side (left turn)
    [~, idx] = min(abs(sw_angle_deg - key_sw_ack(k)));
    if ~isnan(pct_ackermann(idx))
        fprintf('%12.1f  %15.2f%%\n', key_sw_ack(k), pct_ackermann(idx))
    else
        fprintf('%12.1f  %15s\n', key_sw_ack(k), 'below threshold')
    end
end
fprintf('\nOverall range:  %.1f%% to %.1f%%\n', ...
    min(pct_ackermann(~isnan(pct_ackermann))), ...
    max(pct_ackermann(~isnan(pct_ackermann))))

%% -----------------------------------------------------------------------
%  POLYNOMIAL APPROXIMATION
%% -----------------------------------------------------------------------
poly_degree = 4;

valid_R = ~isnan(SR_right) & ~isinf(SR_right);
valid_L = ~isnan(SR_left)  & ~isinf(SR_left);

sw_fit_R = sw_angle_deg(valid_R);  sr_fit_R = SR_right(valid_R);
sw_fit_L = sw_angle_deg(valid_L);  sr_fit_L = SR_left(valid_L);

p_R = polyfit(sw_fit_R, sr_fit_R, poly_degree);
p_L = polyfit(sw_fit_L, sr_fit_L, poly_degree);

sw_dense  = linspace(min(sw_angle_deg), max(sw_angle_deg), 500);
sr_poly_R = polyval(p_R, sw_dense);
sr_poly_L = polyval(p_L, sw_dense);

ss_res_R = sum((sr_fit_R - polyval(p_R, sw_fit_R)).^2);
ss_tot_R = sum((sr_fit_R - mean(sr_fit_R)).^2);
R2_R     = 1 - ss_res_R / ss_tot_R;

ss_res_L = sum((sr_fit_L - polyval(p_L, sw_fit_L)).^2);
ss_tot_L = sum((sr_fit_L - mean(sr_fit_L)).^2);
R2_L     = 1 - ss_res_L / ss_tot_L;

fprintf('\n===== POLYNOMIAL APPROXIMATION: SR vs SW Angle =====\n')
fprintf('Polynomial degree: %d\n', poly_degree)
fprintf('--- RIGHT wheel ---\n')
for k = 1:length(p_R)
    fprintf('  p(%d) = %+.6e\n', k, p_R(k))
end
fprintf('R2 = %.6f\n\n', R2_R)
fprintf('--- LEFT wheel ---\n')
for k = 1:length(p_L)
    fprintf('  p(%d) = %+.6e\n', k, p_L(k))
end
fprintf('R2 = %.6f\n\n', R2_L)

key_sw = [-270, -180, -90, 0, 90, 180, 270];
key_sw = key_sw(key_sw >= min(sw_angle_deg) & key_sw <= max(sw_angle_deg));
fprintf('--- Sample SR values ---\n')
fprintf('%12s  %12s  %12s\n', 'SW [deg]', 'SR_R (fit)', 'SR_L (fit)')
for k = 1:length(key_sw)
    fprintf('%12.1f  %12.3f  %12.3f\n', key_sw(k), polyval(p_R, key_sw(k)), polyval(p_L, key_sw(k)))
end

%% -----------------------------------------------------------------------
%  BICYCLE MODEL: Average Steering Ratio Polynomial Approximation
%% -----------------------------------------------------------------------

% Average SR across both wheels (bicycle model — single front wheel)
SR_avg = (SR_right + SR_left) / 2;

% Only use points where both wheels have valid SR
valid_avg = ~isnan(SR_right) & ~isinf(SR_right) & ...
            ~isnan(SR_left)  & ~isinf(SR_left);

sw_fit_avg = sw_angle_deg(valid_avg);
sr_fit_avg = SR_avg(valid_avg);

% Polynomial fit — same degree as individual fits
poly_degree_avg = poly_degree;   % inherits from earlier (= 4)
p_avg = polyfit(sw_fit_avg, sr_fit_avg, poly_degree_avg);

% Dense evaluation for smooth curve
sr_poly_avg = polyval(p_avg, sw_dense);

% R² goodness of fit
ss_res_avg = sum((sr_fit_avg - polyval(p_avg, sw_fit_avg)).^2);
ss_tot_avg = sum((sr_fit_avg - mean(sr_fit_avg)).^2);
R2_avg     = 1 - ss_res_avg / ss_tot_avg;

% ---- Print coefficients -------------------------------------------------
fprintf('\n===== BICYCLE MODEL: Average SR Polynomial =====\n')
fprintf('SR_avg(SW) = p(1)*SW^%d + p(2)*SW^%d + ... + p(%d)\n', ...
        poly_degree_avg, poly_degree_avg-1, poly_degree_avg+1)
fprintf('Polynomial degree: %d  |  R² = %.6f\n\n', poly_degree_avg, R2_avg)
for k = 1:length(p_avg)
    fprintf('  p_avg(%d) = %+.6e\n', k, p_avg(k))
end

% Convenience: anonymous function ready to paste into vehicle model
fprintf('\nAnonymous function (SW in deg → SR [-]):\n')
coeff_str = '';
for k = 1:length(p_avg)
    pw = poly_degree_avg - k + 1;
    if pw > 1
        coeff_str = [coeff_str, sprintf('(%+.6e).*SW.^%d + ', p_avg(k), pw)]; %#ok<AGROW>
    elseif pw == 1
        coeff_str = [coeff_str, sprintf('(%+.6e).*SW + ',     p_avg(k))];     %#ok<AGROW>
    else
        coeff_str = [coeff_str, sprintf('(%+.6e)',             p_avg(k))];     %#ok<AGROW>
    end
end
fprintf('SR_avg = @(SW) %s;\n\n', coeff_str)

% Sample values at key SW angles
fprintf('--- Sample values ---\n')
fprintf('%12s  %12s  %12s  %12s\n', 'SW [deg]', 'SR_R', 'SR_L', 'SR_avg (fit)')
for k = 1:length(key_sw)
    fprintf('%12.1f  %12.3f  %12.3f  %12.3f\n', ...
        key_sw(k), ...
        polyval(p_R,   key_sw(k)), ...
        polyval(p_L,   key_sw(k)), ...
        polyval(p_avg, key_sw(k)))
end

% Through-centre value
SR_avg_centre = polyval(p_avg, 0);
fprintf('\nSR_avg through centre (SW = 0): %.4f\n', SR_avg_centre)

%% -----------------------------------------------------------------------
%  FIGURE 8b: Bicycle Model Average SR
%% -----------------------------------------------------------------------
figure('Name', 'Figure - Bicycle Model Average Steering Ratio')

% Top panel: SR comparison
axes('position', [0.08, 0.60, 0.86, 0.28])
hold on, grid on
title({['Bicycle Model — Average Steering Ratio  |  Degree-', ...
        num2str(poly_degree_avg), ' Polynomial Fit  |  R² = ', ...
        sprintf('%.5f', R2_avg)]; 'Design ride height  (\phi = 0)'}, ...
    'FontWeight', 'bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)')
ylabel('Steering ratio  SR  [-]')
plot(sw_angle_deg(valid_R),   SR_right(valid_R),   'b.',  'MarkerSize', 8, ...
    'DisplayName', 'SR_{right}  (raw)')
plot(sw_angle_deg(valid_L),   SR_left(valid_L),    'r.',  'MarkerSize', 8, ...
    'DisplayName', 'SR_{left}  (raw)')
plot(sw_angle_deg(valid_avg), SR_avg(valid_avg),   'ko',  'MarkerSize', 5, ...
    'DisplayName', 'SR_{avg}  (raw)')
plot(sw_dense, sr_poly_R,   'b--', 'LineWidth', 1.2, ...
    'DisplayName', 'SR_{right}  poly')
plot(sw_dense, sr_poly_L,   'r--', 'LineWidth', 1.2, ...
    'DisplayName', 'SR_{left}  poly')
plot(sw_dense, sr_poly_avg, 'k-',  'LineWidth', 2.5, ...
    'DisplayName', sprintf('SR_{avg}  poly  (bicycle model)'))
plot(0, SR_avg_centre, 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', ...
    'DisplayName', sprintf('Centre SR_{avg} = %.3f', SR_avg_centre))
xline(0, 'k:', 'HandleVisibility', 'off')
yline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

% Middle panel: road wheel angle (bicycle = average of both)
del_avg_deg = (d1_deg + d2_deg) / 2;

axes('position', [0.08, 0.33, 0.86, 0.20])
hold on, grid on
title({'Bicycle Model Front Wheel Steer Angle  (\delta_{avg}) vs Steering Wheel Angle'}, ...
    'FontWeight', 'bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)')
ylabel('\delta_{avg}  [deg]  (+: left steer)')
plot(sw_angle_deg, del_avg_deg, 'k-', 'LineWidth', 2, ...
    'DisplayName', '\delta_{avg} = (\delta_R + \delta_L)/2')
plot(sw_angle_deg, d1_deg, 'b--', 'LineWidth', 1, 'DisplayName', '\delta_{right}')
plot(sw_angle_deg, d2_deg, 'r--', 'LineWidth', 1, 'DisplayName', '\delta_{left}')
xline(0, 'k:', 'HandleVisibility', 'off')
yline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

% Bottom panel: residuals
axes('position', [0.08, 0.10, 0.86, 0.16])
hold on, grid on
title('SR_{avg} Residuals  (raw - fitted)', 'FontWeight', 'bold')
xlabel('Steering wheel angle  [deg]')
ylabel('Residual  [-]')
stem(sw_fit_avg, sr_fit_avg - polyval(p_avg, sw_fit_avg), 'k', ...
    'filled', 'MarkerSize', 4, 'DisplayName', 'Residual SR_{avg}')
yline(0, 'k-', 'LineWidth', 1, 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

bicy_ribbon = {
    ['Bicycle model SR: arithmetic mean of right and left road wheel steering ratios'], ...
    ['Degree-', num2str(poly_degree_avg), ' polynomial fit  |  R² = ', ...
     sprintf('%.5f', R2_avg), '  |  SR_avg at centre = ', ...
     sprintf('%.4f', SR_avg_centre)], ...
    'SR(SW) = SW_angle / delta_avg  —  single equivalent front wheel for vehicle dynamics', ...
    'Coefficients and anonymous function printed in Command Window'
};
add_sign_box(bicy_ribbon, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 7: Polynomial Fit  (was Figure 6)
%% -----------------------------------------------------------------------
figure('Name', 'Figure 7 - Polynomial Fit: Steering Ratio vs SW Angle')

axes('position', [0.08, 0.72, 0.86, 0.20])
hold on, grid on
title({['Steering Ratio vs Steering Wheel Angle  |  Degree-', num2str(poly_degree), ' Polynomial Fit']; 'Design ride height  (\phi = 0)'}, 'FontWeight', 'bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)'), ylabel('Steering ratio  SR  [-]')
plot(sw_angle_deg(valid_R), SR_right(valid_R), 'bo', 'MarkerSize', 6, 'DisplayName', 'SR_{right}  (raw)')
plot(sw_angle_deg(valid_L), SR_left(valid_L),  'rs', 'MarkerSize', 6, 'DisplayName', 'SR_{left}  (raw)')
plot(sw_dense, sr_poly_R, 'b-', 'LineWidth', 2.0, 'DisplayName', sprintf('SR_{right}  poly (R^2=%.4f)', R2_R))
plot(sw_dense, sr_poly_L, 'r--','LineWidth', 2.0, 'DisplayName', sprintf('SR_{left}   poly (R^2=%.4f)', R2_L))
xline(0, 'k:', 'HandleVisibility', 'off'), yline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

axes('position', [0.08, 0.42, 0.86, 0.20])
hold on, grid on
title({'Road Wheel Steer Angles — Lock to Lock'; 'Design ride height  (\phi = 0)'}, 'FontWeight', 'bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)'), ylabel('Road wheel angle  \delta  [deg]  (+: left steer)')
plot(sw_angle_deg, d1_deg, 'b-',  'LineWidth', 2.0, 'DisplayName', 'Right wheel  \delta_{right}')
plot(sw_angle_deg, d2_deg, 'r--', 'LineWidth', 2.0, 'DisplayName', 'Left wheel  \delta_{left}')
xline(0, 'k:', 'HandleVisibility', 'off'), yline(0, 'k:', 'HandleVisibility', 'off')
[~, idx_min] = min(sw_angle_deg);  [~, idx_max] = max(sw_angle_deg);
sw_min = min(sw_angle_deg);        sw_max = max(sw_angle_deg);
text(sw_min, d1_deg(idx_min), sprintf('  R: %.1f°', d1_deg(idx_min)), 'FontSize', 8, 'Color', 'b', 'VerticalAlignment', 'bottom')
text(sw_min, d2_deg(idx_min), sprintf('  L: %.1f°', d2_deg(idx_min)), 'FontSize', 8, 'Color', 'r', 'VerticalAlignment', 'top')
text(sw_max, d1_deg(idx_max), sprintf('R: %.1f°  ', d1_deg(idx_max)), 'FontSize', 8, 'Color', 'b', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom')
text(sw_max, d2_deg(idx_max), sprintf('L: %.1f°  ', d2_deg(idx_max)), 'FontSize', 8, 'Color', 'r', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top')
text(0, max([d1_deg, d2_deg])*0.85, sprintf('Lock-to-lock SW: %.1f° to %.1f° (%.1f° total)', sw_min, sw_max, sw_max-sw_min), 'FontSize', 8.5, 'HorizontalAlignment', 'center', 'FontWeight', 'bold')
legend('Location', 'best', 'FontSize', 9)

axes('position', [0.08, 0.13, 0.86, 0.20])
hold on, grid on
title('Residuals  (raw - fitted)', 'FontWeight', 'bold')
xlabel('Steering wheel angle  [deg]'), ylabel('SR residual  [-]')
stem(sw_fit_R, sr_fit_R - polyval(p_R, sw_fit_R), 'b', 'filled', 'MarkerSize', 4, 'DisplayName', 'Residual  R')
stem(sw_fit_L, sr_fit_L - polyval(p_L, sw_fit_L), 'r', 'filled', 'MarkerSize', 4, 'DisplayName', 'Residual  L')
yline(0, 'k-', 'LineWidth', 1, 'HandleVisibility', 'off'), xline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

poly_ribbon = {
    ['Polynomial degree: ', num2str(poly_degree), '  |  R2_right = ', num2str(R2_R,'%.5f'), '  |  R2_left  = ', num2str(R2_L,'%.5f')], ...
    ['Lock-to-lock: SW ', num2str(sw_min,'%.1f'), ' to ', num2str(sw_max,'%.1f'), '  |  Right: ', num2str(d1_deg(idx_min),'%.2f'), ' to ', num2str(d1_deg(idx_max),'%.2f'), '  |  Left: ', num2str(d2_deg(idx_min),'%.2f'), ' to ', num2str(d2_deg(idx_max),'%.2f')], ...
    'SR = p(1)*SW^n + ... + p(n+1)   [coefficients printed in Command Window]', ...
    'Residual plot shows goodness-of-fit — ideal: residuals scattered randomly around zero'
};
add_sign_box(poly_ribbon, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 8: Cornering Camber 
%% -----------------------------------------------------------------------
lat_accel_G      = 1.4;
roll_gradient    = 0.28;
steer_wheel_deg  = 70;

roll_angle_deg = lat_accel_G * roll_gradient;
roll_angle_rad = roll_angle_deg / 180 * pi;

phi_outer_deg =  roll_angle_deg / 2;
phi_inner_deg = -roll_angle_deg / 2;
phi_outer_rad =  phi_outer_deg / 180 * pi;
phi_inner_rad =  phi_inner_deg / 180 * pi;

u_steer_inch = -steer_wheel_deg * c_factor_in_per_rev / 360;
u_steer_m    = u_steer_inch / 39.3701;

fprintf('\n===== CORNERING CONDITION =====\n')
fprintf('Lateral acceleration:   %.2f G\n',    lat_accel_G)
fprintf('Roll gradient:          %.3f deg/G\n', roll_gradient)
fprintf('Total roll angle:       %.4f deg\n',   roll_angle_deg)
fprintf('Outer (right) phi:     +%.4f deg\n',   phi_outer_deg)
fprintf('Inner (left)  phi:      %.4f deg\n',   phi_inner_deg)
fprintf('Steering wheel angle:   %.1f deg  (+ = left turn)\n', steer_wheel_deg)
fprintf('Rack displacement:      %.4f mm\n',    u_steer_m*1000)

par_left = par;
fields_f = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk','rvrk','rvqk'};
for f = 1:length(fields_f)
    par_left.(fields_f{f})(2) = -par.(fields_f{f})(2);
end

[avw_design_r, ~, ~, ~] = fun_05_dblwb_kin(0, 0, par);
eyrv_design_r = avw_design_r * eyrk;
camb_design_r = atan2(eyrv_design_r(3), eyrv_design_r(2)) * 180/pi;

[avw_design_l, ~, ~, ~] = fun_05_dblwb_kin(0, 0, par_left);
eyrv_design_l = avw_design_l * eyrk;
camb_design_l = atan2(eyrv_design_l(3), eyrv_design_l(2)) * 180/pi;

fprintf('\nDesign camber - Right: %+.4f deg\n', camb_design_r)
fprintf('Design camber - Left:  %+.4f deg\n',  camb_design_l)

[avw_outer, ~, ~, ~] = fun_05_dblwb_kin(phi_outer_rad, u_steer_m, par);
eyrv_outer     = avw_outer * eyrk;
camb_outer_abs = atan2(eyrv_outer(3), eyrv_outer(2)) * 180/pi;
camb_outer_chg = camb_outer_abs - camb_design_r;

[avw_inner, ~, ~, ~] = fun_05_dblwb_kin(phi_inner_rad, u_steer_m, par_left);
eyrv_inner     = avw_inner * eyrk;
camb_inner_abs = atan2(eyrv_inner(3), eyrv_inner(2)) * 180/pi;
camb_inner_chg = camb_inner_abs - camb_design_l;

camb_outer_road = camb_outer_chg - roll_angle_deg / 2;
camb_inner_road = camb_inner_chg + roll_angle_deg / 2;

[~, ~, delta_outer,     ~] = fun_05_dblwb_kin(phi_outer_rad, u_steer_m, par);
[~, ~, delta_inner_raw, ~] = fun_05_dblwb_kin(phi_inner_rad, u_steer_m, par_left);
delta_inner = delta_inner_raw;

fprintf('\n===== STEERING ANGLE VALIDATION =====\n')
fprintf('Outer wheel steer: %+.4f deg\n', delta_outer*180/pi)
fprintf('Inner wheel steer: %+.4f deg\n', delta_inner*180/pi)
fprintf('Both steer left:   %s\n', ternary_str(delta_outer>0 && delta_inner>0, 'PASS', 'FAIL'))
fprintf('Ackermann:         %s\n', ternary_str(delta_inner>delta_outer, sprintf('PASS (%.4f > %.4f)', delta_inner*180/pi, delta_outer*180/pi), sprintf('FAIL (%.4f < %.4f)', delta_inner*180/pi, delta_outer*180/pi)))

fprintf('\n===== CAMBER AT CORNERING CONDITION =====\n')
fprintf('Outer suspension camber change:  %+.4f deg\n', camb_outer_chg)
fprintf('Outer road-relative camber:      %+.4f deg\n', camb_outer_road)
fprintf('Inner suspension camber change:  %+.4f deg\n', camb_inner_chg)
fprintf('Inner road-relative camber:      %+.4f deg\n', camb_inner_road)

G_range          = linspace(0, lat_accel_G, 50);
camb_outer_sweep = zeros(1, 50);
camb_inner_sweep = zeros(1, 50);

for k = 1:length(G_range)
    g     = G_range(k);
    phi_o =  (g * roll_gradient / 2) / 180 * pi;
    phi_i = -(g * roll_gradient / 2) / 180 * pi;
    u_k   = -(steer_wheel_deg * g / lat_accel_G) * c_factor_in_per_rev / 360 / 39.3701;

    [avw_o, ~, ~, ~] = fun_05_dblwb_kin(phi_o, u_k, par);
    eyrv_o = avw_o * eyrk;
    camb_outer_sweep(k) = atan2(eyrv_o(3), eyrv_o(2)) * 180/pi - camb_design_r;

    [avw_i, ~, ~, ~] = fun_05_dblwb_kin(phi_i, u_k, par_left);
    eyrv_i = avw_i * eyrk;
    camb_inner_sweep(k) = atan2(eyrv_i(3), eyrv_i(2)) * 180/pi - camb_design_l;
end

figure('Name', 'Figure 8 - Camber Change in Roll  |  Front Axle Cornering')

axes('position', [0.07, 0.55, 0.40, 0.35])
hold on, grid on
title({'Camber Change vs Lateral Acceleration  |  LEFT turn'; ...
       ['Front Axle  |  Roll gradient = ', num2str(roll_gradient), ' deg/G  |  SW = +', num2str(steer_wheel_deg), '°  (+ = left turn)']}, 'FontWeight','bold')
xlabel('Lateral acceleration  [G]'), ylabel('\Delta\gamma  [deg]  (+: top outboard)')
plot(G_range, camb_outer_sweep, 'b-',  'LineWidth', 2, 'DisplayName', 'Outer wheel (right, jounce)')
plot(G_range, camb_inner_sweep, 'r--', 'LineWidth', 2, 'DisplayName', 'Inner wheel (left, droop)')
plot(lat_accel_G, camb_outer_chg, 'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', sprintf('Outer @ %.1fG: %+.3f°', lat_accel_G, camb_outer_chg))
plot(lat_accel_G, camb_inner_chg, 'rv', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', sprintf('Inner @ %.1fG: %+.3f°', lat_accel_G, camb_inner_chg))
xline(lat_accel_G, 'k:', 'HandleVisibility', 'off'), yline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

axes('position', [0.57, 0.55, 0.38, 0.35])
hold on, grid on
title({['Camber Relative to Road  |  ', num2str(lat_accel_G), 'G  LEFT turn']; ...
       ['Roll = ', num2str(roll_angle_deg, '%.3f'), '°  |  SW = +', num2str(steer_wheel_deg), '°  (+ = left turn)']}, 'FontWeight', 'bold')
ylabel('\Delta\gamma_{road}  [deg]  (+: top outboard)')
road_vals = [camb_outer_road, camb_inner_road];
bar_cols  = [0.2 0.4 0.8; 0.8 0.2 0.2];
for b = 1:2
    bar(b, road_vals(b), 0.5, 'FaceColor', bar_cols(b,:), 'EdgeColor', 'none')
end
set(gca, 'XTick', 1:2, 'XTickLabel', {'Outer (Right)', 'Inner (Left)'})
yline(0, 'k-', 'LineWidth', 1, 'HandleVisibility', 'off')
y_pad = max(abs(road_vals)) * 0.20;
ylim([min(road_vals) - y_pad, max(road_vals) + y_pad])
for b = 1:2
    if road_vals(b) < 0
        text(b, road_vals(b) - y_pad*0.3, sprintf('%+.3f°', road_vals(b)), 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold')
    else
        text(b, road_vals(b) + y_pad*0.3, sprintf('%+.3f°', road_vals(b)), 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold')
    end
end

axes('position', [0.07, 0.10, 0.88, 0.33])
axis off
title('Cornering Camber Summary', 'FontWeight', 'bold')
sum_headers = {'Parameter', 'Outer wheel (Right)', 'Inner wheel (Left)'};
sum_col_x   = [0.02, 0.38, 0.72];
row_y_s = 0.88;  row_dy_s = 0.115;
for c = 1:length(sum_headers)
    text(sum_col_x(c), row_y_s, sum_headers{c}, 'FontSize', 9, 'FontWeight', 'bold', 'Interpreter', 'none')
end
annotation('line', [0.07, 0.95], [0.385, 0.385], 'Color', 'k', 'LineWidth', 1.2)
sum_rows = {
    'Wheel side',                  'Right (outer)',                             'Left (inner)';
    'Suspension motion',           sprintf('Jounce  %+.4f deg', phi_outer_deg), sprintf('Droop   %.4f deg', phi_inner_deg);
    'Roll contribution',           sprintf('%+.4f deg', -roll_angle_deg/2),      sprintf('%+.4f deg', +roll_angle_deg/2);
    'Suspension \Delta camber',    sprintf('%+.4f deg', camb_outer_chg),         sprintf('%+.4f deg', camb_inner_chg);
    'Road-relative \Delta camber', sprintf('%+.4f deg', camb_outer_road),        sprintf('%+.4f deg', camb_inner_road);
    'Road wheel steer angle',      sprintf('%+.4f deg', delta_outer*180/pi),     sprintf('%+.4f deg', delta_inner*180/pi);
    'Rack displacement',           sprintf('%+.4f mm',  u_steer_m*1000),         sprintf('%+.4f mm',  u_steer_m*1000);
};
for r = 1:size(sum_rows, 1)
    ry_s = row_y_s - r * row_dy_s;
    if mod(r, 2) == 0
        patch([0, 1, 1, 0], [ry_s-row_dy_s*0.4, ry_s-row_dy_s*0.4, ry_s+row_dy_s*0.6, ry_s+row_dy_s*0.6], [0.93 0.93 0.93], 'EdgeColor', 'none', 'FaceAlpha', 0.6)
    end
    for c = 1:3
        text(sum_col_x(c), ry_s, sum_rows{r, c}, 'FontSize', 8.5, 'Interpreter', 'none')
    end
end

corner_ribbon = {
    ['Lateral accel: ', num2str(lat_accel_G), ' G  |  Roll gradient: ', num2str(roll_gradient), ' deg/G  |  Total roll: ', num2str(roll_angle_deg, '%.3f'), ' deg'], ...
    ['SW angle: +', num2str(steer_wheel_deg), '° = LEFT turn  |  Right wheel = OUTER (jounce)  |  Left wheel = INNER (droop)  |  Rack disp: ', num2str(u_steer_m*1000, '%.3f'), ' mm'], ...
    'Camber (+): top outboard  |  Road-relative camber = suspension change + body roll contribution', ...
    'Roll split assumed 50/50 between axle sides  |  Body roll adds negative camber to outer, positive to inner'
};
add_sign_box(corner_ribbon, wheel_side)

%% -----------------------------------------------------------------------
%  ROLL CENTRE ESTIMATION
%% -----------------------------------------------------------------------
roll_gradient_rc  = 0.28;
lat_accel_rc      = 1.0;
roll_angle_rc_deg = lat_accel_rc * roll_gradient_rc;

phi_rc_outer =  (roll_angle_rc_deg / 2) / 180 * pi;
phi_rc_inner = -(roll_angle_rc_deg / 2) / 180 * pi;

par_rc_l = par;
fields_rc = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk','rvrk','rvqk'};
for f = 1:length(fields_rc)
    par_rc_l.(fields_rc{f})(2) = -par.(fields_rc{f})(2);
end

% Build phi sweep with exact zero — finer resolution for small roll angles
n_rc_half    = 100;
phi_rc_neg   = linspace(-par.phmx_droop,   0, n_rc_half);
phi_rc_pos   = linspace(0, par.phmx_jounce, n_rc_half);
phi_rc_sweep = [phi_rc_neg(1:end-1), phi_rc_pos];
n_rc         = length(phi_rc_sweep);
rc_height    = zeros(1, n_rc);

IC_r_y_sweep = zeros(1, n_rc);
IC_r_z_sweep = zeros(1, n_rc);
CP_r_y_sweep = zeros(1, n_rc);
CP_r_z_sweep = zeros(1, n_rc);

% -------------------------------------------------------------------------
%  BODY ROLL METHOD — inboard points move with body roll
%  Outboard points follow kinematic solution
%  Each phi_k represents half the total body roll angle
% -------------------------------------------------------------------------
rc_y      = zeros(1, n_rc);
rc_height = zeros(1, n_rc);

yw_rc_r = zeros(1, n_rc);  zw_rc_r = zeros(1, n_rc);
yw_rc_l = zeros(1, n_rc);  zw_rc_l = zeros(1, n_rc);

for k = 1:n_rc
    phi_k = phi_rc_sweep(k);   % half total roll angle [rad]
    % Total body roll = 2 * phi_k
    % Right wheel = jounce (+phi_k), Left wheel = droop (-phi_k)

    % -----------------------------------------------------------------
    %  BODY ROLL ROTATION MATRIX about X axis (roll axis)
    %  Right side: body rolls by +phi_k (right side goes down = jounce)
    %  Left side:  body rolls by +phi_k (left side goes up  = droop)
    %  Rotation matrix in YZ plane for angle alpha:
    %  [Y']   [cos(a)  -sin(a)] [Y]
    %  [Z'] = [sin(a)   cos(a)] [Z]
    % -----------------------------------------------------------------
    alpha = phi_k;   % body roll angle for this step

    R_roll = [cos(alpha), -sin(alpha); ...
              sin(alpha),  cos(alpha)];

    % -----------------------------------------------------------------
    %  RIGHT WHEEL — inboard points rotated by +alpha (right side down)
    % -----------------------------------------------------------------
    % Rotate inboard chassis pivots about vehicle centreline (Y=0)
    % Lower arm chassis pivot (midpoint of A and B)
    rvab_r0  = (par.rvak + par.rvbk) / 2;
    rvab_r0_yz = [rvab_r0(2); rvab_r0(3)];
    rvab_r_yz  = R_roll * rvab_r0_yz;   % rotated inboard lower pivot

    rvde_r0  = (par.rvdk + par.rvek) / 2;
    rvde_r0_yz = [rvde_r0(2); rvde_r0(3)];
    rvde_r_yz  = R_roll * rvde_r0_yz;   % rotated inboard upper pivot

    % Outboard points from kinematic solution (wheel follows suspension)
    [~, rvwv_r, ~, ~] = fun_05_dblwb_kin(phi_k, 0, par);

    % For outboard pivot positions, use kinematic function with
    % small perturbation to find deflected C and F positions
    % C (lower outboard) and F (upper outboard) move with the wheel
    % Approximate: rotate outboard points by camber change
    % Better: use static outboard hardpoints rotated by phi_k
    % (they are fixed to the upright which rotates with suspension)
    rvc_r0_yz  = [par.rvck(2); par.rvck(3)];
    rvf_r0_yz  = [par.rvfk(2); par.rvfk(3)];

    % Outboard points rotate with wheel (heave motion shifts them in Z)
    % Use the wheel centre Z shift to translate outboard points
    dZ_r = rvwv_r(3) - par.rvwk(3);
    dY_r = rvwv_r(2) - par.rvwk(2);

    rvc_r_yz = [rvc_r0_yz(1) + dY_r; rvc_r0_yz(2) + dZ_r];
    rvf_r_yz = [rvf_r0_yz(1) + dY_r; rvf_r0_yz(2) + dZ_r];

    % IC right = intersection of lower arm line and upper arm line
    d1_r = rvc_r_yz - rvab_r_yz;
    d2_r = rvf_r_yz - rvde_r_yz;

    cross_r = d1_r(1)*d2_r(2) - d1_r(2)*d2_r(1);

    if abs(cross_r) < 1e-10
        IC_r = [Inf; Inf];
    else
        diff_r = rvde_r_yz - rvab_r_yz;
        t_r    = (diff_r(1)*d2_r(2) - diff_r(2)*d2_r(1)) / cross_r;
        IC_r   = rvab_r_yz + t_r * d1_r;
    end

    CP_r = [rvwv_r(2); rvwv_r(3) - rs];

    % -----------------------------------------------------------------
    %  LEFT WHEEL — inboard points rotated by +alpha (left side up)
    %  For left wheel, body rolling right means left side lifts (droop)
    %  Inboard points on left also rotate by +alpha about centreline
    % -----------------------------------------------------------------
    rvab_l0  = (par_rc_l.rvak + par_rc_l.rvbk) / 2;
    rvab_l0_yz = [rvab_l0(2); rvab_l0(3)];
    rvab_l_yz  = R_roll * rvab_l0_yz;

    rvde_l0  = (par_rc_l.rvdk + par_rc_l.rvek) / 2;
    rvde_l0_yz = [rvde_l0(2); rvde_l0(3)];
    rvde_l_yz  = R_roll * rvde_l0_yz;

    [~, rvwv_l, ~, ~] = fun_05_dblwb_kin(-phi_k, 0, par_rc_l);

    rvc_l0_yz  = [par_rc_l.rvck(2); par_rc_l.rvck(3)];
    rvf_l0_yz  = [par_rc_l.rvfk(2); par_rc_l.rvfk(3)];

    dZ_l = rvwv_l(3) - par_rc_l.rvwk(3);
    dY_l = rvwv_l(2) - par_rc_l.rvwk(2);

    rvc_l_yz = [rvc_l0_yz(1) + dY_l; rvc_l0_yz(2) + dZ_l];
    rvf_l_yz = [rvf_l0_yz(1) + dY_l; rvf_l0_yz(2) + dZ_l];

    d1_l = rvc_l_yz - rvab_l_yz;
    d2_l = rvf_l_yz - rvde_l_yz;

    cross_l = d1_l(1)*d2_l(2) - d1_l(2)*d2_l(1);

    if abs(cross_l) < 1e-10
        IC_l = [Inf; Inf];
    else
        diff_l = rvde_l_yz - rvab_l_yz;
        t_l    = (diff_l(1)*d2_l(2) - diff_l(2)*d2_l(1)) / cross_l;
        IC_l   = rvab_l_yz + t_l * d1_l;
    end

    CP_l = [rvwv_l(2); rvwv_l(3) - rs];

    % Store
    CP_r_y_sweep(k) = CP_r(1);
    CP_r_z_sweep(k) = CP_r(2);
    yw_rc_r(k) = rvwv_r(2);  zw_rc_r(k) = rvwv_r(3);
    yw_rc_l(k) = rvwv_l(2);  zw_rc_l(k) = rvwv_l(3);
    IC_r_y_sweep(k) = IC_r(1);
    IC_r_z_sweep(k) = IC_r(2);

    % -----------------------------------------------------------------
    %  RC = intersection of CP_r->IC_r and CP_l->IC_l lines
    % -----------------------------------------------------------------
    if any(isinf(IC_r)) || any(isinf(IC_l))
        rc_y(k)      = 0;
        rc_height(k) = 0;
        continue
    end

    dir_r = IC_r - CP_r;
    dir_l = IC_l - CP_l;

    A     = [dir_r(1), -dir_l(1); dir_r(2), -dir_l(2)];
    b_vec = CP_l - CP_r;
    det_A = A(1,1)*A(2,2) - A(1,2)*A(2,1);

    if abs(det_A) < 1e-12
        rc_y(k)      = 0;
        rc_height(k) = (CP_r(2) + CP_l(2)) / 2;
        continue
    end

    t_sol        = (b_vec(1)*A(2,2) - b_vec(2)*A(1,2)) / det_A;
    RC_pt        = CP_r + t_sol * dir_r;
    rc_y(k)      = RC_pt(1);
    rc_height(k) = RC_pt(2);
end

% -------------------------------------------------------------------------
%  KEY INDICES AND VALUES
% -------------------------------------------------------------------------
[~, idx_des_rc] = min(abs(phi_rc_sweep));
[~, idx_1g]     = min(abs(phi_rc_sweep - phi_rc_outer));

rc_height_design  = rc_height(idx_des_rc);
rc_height_1g      = rc_height(idx_1g);
rc_vert_migration = (rc_height_1g - rc_height_design) * 1000;

rc_y_design       = rc_y(idx_des_rc);
rc_y_1g           = rc_y(idx_1g);
rc_lat_migration  = (rc_y_1g - rc_y_design) * 1000;

IC_r_y_design  = IC_r_y_sweep(idx_des_rc);
IC_r_z_design  = IC_r_z_sweep(idx_des_rc);
IC_r_y_1g      = IC_r_y_sweep(idx_1g);
IC_r_z_1g      = IC_r_z_sweep(idx_1g);
CP_r_y_design  = CP_r_y_sweep(idx_des_rc);
CP_r_y_1g      = CP_r_y_sweep(idx_1g);

IC_y_migration = (IC_r_y_1g - IC_r_y_design) * 1000;
CP_y_migration = (CP_r_y_1g - CP_r_y_design) * 1000;

heave_rc_mm = interp1(phi(1:n), (zw(:,m0) - zw(n0,m0))*1000, ...
                      phi_rc_sweep, 'linear', 'extrap');

fprintf('\n===== RC SLOPE METHOD VALIDATION =====\n')
fprintf('At design (phi=0): RC_y should be ~0 for symmetric geometry\n')
fprintf('RC_y at design:    %+.4f mm\n', rc_y_design*1000)
fprintf('RC_z at design:    %.4f mm\n',  rc_height_design*1000)
fprintf('\nAt 1G outer phi = +%.4f deg:\n', phi_rc_outer*180/pi)
fprintf('RC_y at 1G:        %+.4f mm\n', rc_y_1g*1000)
fprintf('RC_z at 1G:        %.4f mm\n',  rc_height_1g*1000)
fprintf('\n===== ROLL CENTRE MIGRATION =====\n')
fprintf('RC vertical migration:    %+.4f mm\n', rc_vert_migration)
fprintf('RC lateral migration:     %+.4f mm\n', rc_lat_migration)
fprintf('IC lateral migration:     %+.4f mm\n', IC_y_migration)
fprintf('CP lateral migration:     %+.4f mm\n', CP_y_migration)

% Convert phi sweep to heave mm
heave_rc_mm = interp1(phi(1:n), (zw(:,m0) - zw(n0,m0))*1000, ...
                      phi_rc_sweep, 'linear', 'extrap');

fprintf('\n===== ROLL CENTRE ESTIMATION =====\n')
fprintf('Design RC height:         %.2f mm\n',  rc_height_design*1000)
fprintf('RC height at 1G:          %.2f mm\n',  rc_height_1g*1000)
fprintf('RC vertical migration:    %+.2f mm\n', rc_vert_migration)
fprintf('\nIC right - Design:        Y = %+.2f mm,  Z = %.2f mm\n', IC_r_y_design*1000, IC_r_z_design*1000)
fprintf('IC right - At 1G:         Y = %+.2f mm,  Z = %.2f mm\n', IC_r_y_1g*1000,     IC_r_z_1g*1000)
fprintf('IC lateral migration:     %+.2f mm\n', IC_y_migration)
fprintf('CP lateral migration:     %+.2f mm\n', CP_y_migration)
fprintf('\nNote: geometric RC — no compliance or tyre deflection\n')

% -------------------------------------------------------------------------
%  FIGURE
% -------------------------------------------------------------------------
figure('Name', ['Roll Centre Migration  |  Front Axle  |  ', ...
    num2str(lat_accel_rc), 'G LEFT corner  (lateral accel toward +Y)'])

% Top-left: RC height vs heave
axes('position', [0.05, 0.57, 0.27, 0.35])
hold on, grid on
title({'RC Height vs Heave  |  Front Axle'; ...
       [num2str(lat_accel_rc), 'G LEFT corner  |  Right = outer (jounce)  |  Left = inner (droop)  |  ', ...
        'Migration: ', sprintf('%+.1f', rc_vert_migration), ' mm']}, ...
    'FontWeight', 'bold')
xlabel('Wheel centre heave  [mm]  (+: jounce)')
ylabel('RC height  [mm]  (+ above ground)')

ylims_rc = [min(rc_height)*1000*0.85, max(rc_height)*1000*1.1];
fill([0, max(heave_rc_mm), max(heave_rc_mm), 0], ...
     [ylims_rc(1), ylims_rc(1), ylims_rc(2), ylims_rc(2)], ...
     [1 0.9 0.9], 'EdgeColor','none','FaceAlpha',0.3,'HandleVisibility','off')
fill([min(heave_rc_mm), 0, 0, min(heave_rc_mm)], ...
     [ylims_rc(1), ylims_rc(1), ylims_rc(2), ylims_rc(2)], ...
     [0.9 0.9 1], 'EdgeColor','none','FaceAlpha',0.3,'HandleVisibility','off')
plot(heave_rc_mm, rc_height*1000, 'b-', 'LineWidth', 2.5, 'DisplayName', 'RC height')
plot(heave_rc_mm(idx_des_rc), rc_height_design*1000, 'g*', 'MarkerSize', 14, 'LineWidth', 2, ...
    'DisplayName', sprintf('Design: %.1f mm', rc_height_design*1000))
plot(heave_rc_mm(idx_1g), rc_height_1g*1000, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('At 1G: %.1f mm', rc_height_1g*1000))
yline(0, 'k--', 'LineWidth', 1, 'DisplayName', 'Ground')
xline(0, 'k:',  'LineWidth', 1, 'HandleVisibility', 'off')
text(max(heave_rc_mm)*0.5,  ylims_rc(1)+(ylims_rc(2)-ylims_rc(1))*0.06, ...
    'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(heave_rc_mm)*0.55, ylims_rc(1)+(ylims_rc(2)-ylims_rc(1))*0.06, ...
    'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
ylim(ylims_rc)
legend('Location', 'northeast', 'FontSize', 9)

% Top-centre: IC and CP lateral migration
axes('position', [0.36, 0.57, 0.17, 0.35])
hold on, grid on
title({'RC & CP Lateral Position vs Heave'; ...
       [num2str(lat_accel_rc), 'G LEFT corner  |  RC mig: ', ...
        sprintf('%+.2f', rc_lat_migration), ' mm  |  CP mig: ', ...
        sprintf('%+.2f', CP_y_migration), ' mm']}, ...
    'FontWeight', 'bold')
xlabel('Heave  [mm]  (+: jounce)')
ylabel('Lateral pos.  [mm]  (+ left of CL)')

cp_pad = max(max(abs(CP_r_y_sweep))*1000 * 0.01, 0.5);
y_lo = min(CP_r_y_sweep)*1000 - cp_pad;
y_hi = max(CP_r_y_sweep)*1000 + cp_pad;

fill([0, max(heave_rc_mm), max(heave_rc_mm), 0], ...
     [y_lo, y_lo, y_hi, y_hi], ...
     [1 0.9 0.9], 'EdgeColor','none','FaceAlpha',0.3,'HandleVisibility','off')
fill([min(heave_rc_mm), 0, 0, min(heave_rc_mm)], ...
     [y_lo, y_lo, y_hi, y_hi], ...
     [0.9 0.9 1], 'EdgeColor','none','FaceAlpha',0.3,'HandleVisibility','off')

plot(heave_rc_mm, CP_r_y_sweep*1000, 'b--', 'LineWidth', 2, 'DisplayName', 'CP Y - Right')
plot(heave_rc_mm, rc_y*1000,         'k-',  'LineWidth', 2, 'DisplayName', 'RC lateral pos.')
plot(heave_rc_mm(idx_des_rc), rc_y_design*1000, 'g*', 'MarkerSize', 12, 'LineWidth', 2, ...
    'DisplayName', sprintf('RC design: %+.2f mm', rc_y_design*1000))
plot(heave_rc_mm(idx_1g), rc_y_1g*1000, 'r^', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('RC at 1G: %+.2f mm', rc_y_1g*1000))
ylim([y_lo, y_hi])
yline(0, 'k--', 'LineWidth', 1, 'DisplayName', 'Centreline')
xline(0, 'k:',  'LineWidth', 1, 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 7.5)

% Top-right: RC height vs IC lateral — migration path
axes('position', [0.57, 0.57, 0.17, 0.35])
hold on, grid on
title({'RC Height vs IC Lateral'; 'Migration path'}, 'FontWeight', 'bold')
xlabel('IC lateral  [mm]  (+ left)')
ylabel('RC height  [mm]')

scatter(IC_r_y_sweep*1000, rc_height*1000, 30, heave_rc_mm, 'filled', ...
    'DisplayName', 'Path (by heave)')
cbar_rc = colorbar;
cbar_rc.Label.String = 'Heave [mm]';
cbar_rc.Label.FontSize = 7;
cbar_rc.FontSize = 7;
cbar_rc.Position(3) = 0.008;

plot(IC_r_y_design*1000, rc_height_design*1000, 'g*', 'MarkerSize', 14, 'LineWidth', 2, ...
    'DisplayName', sprintf('Design (%.1f, %.1f)', IC_r_y_design*1000, rc_height_design*1000))
plot(IC_r_y_1g*1000, rc_height_1g*1000, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('1G (%.1f, %.1f)', IC_r_y_1g*1000, rc_height_1g*1000))
if abs(rc_vert_migration) > 0.01 || abs(IC_y_migration) > 0.01
    quiver(IC_r_y_design*1000, rc_height_design*1000, ...
           IC_y_migration, rc_vert_migration, 0, 'k-', 'LineWidth', 2, ...
           'MaxHeadSize', 0.5, 'DisplayName', ...
           sprintf('Mig (%+.1f, %+.1f) mm', IC_y_migration, rc_vert_migration))
end
yline(0, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 7)

% Summary table
axes('position', [0.78, 0.57, 0.20, 0.35])
axis off
title('RC Summary', 'FontWeight', 'bold')

sum_rc = {
    'Parameter',                 'Value';
    'Design RC height',          sprintf('%.2f mm',  rc_height_design*1000);
    'RC height at 1G',           sprintf('%.2f mm',  rc_height_1g*1000);
    'RC vertical migration',     sprintf('%+.2f mm', rc_vert_migration);
    'RC lateral migration',      sprintf('%+.2f mm', rc_lat_migration);
    'Design IC lateral',         sprintf('%+.2f mm', IC_r_y_design*1000);
    'IC lateral at 1G',          sprintf('%+.2f mm', IC_r_y_1g*1000);
    'IC lateral migration',      sprintf('%+.2f mm', IC_y_migration);
    'CP lateral migration',      sprintf('%+.2f mm', CP_y_migration);
    'Total roll at 1G',          sprintf('%.3f deg', roll_angle_rc_deg);
    'Outer phi (jounce)',        sprintf('+%.3f deg', phi_rc_outer*180/pi);
};

col_x_rc = [0.01, 0.52];
row_y_rc  = 0.92;
row_dy_rc = 0.100;

for c = 1:2
    text(col_x_rc(c), row_y_rc, sum_rc{1,c}, ...
        'FontSize', 8, 'FontWeight', 'bold', 'Interpreter', 'none')
end
for r = 2:size(sum_rc,1)
    ry = row_y_rc - (r-1)*row_dy_rc;
    if mod(r,2) == 0
        patch([0, 1, 1, 0], [ry-row_dy_rc*0.3, ry-row_dy_rc*0.3, ...
               ry+row_dy_rc*0.7, ry+row_dy_rc*0.7], ...
              [0.93 0.93 0.93], 'EdgeColor', 'none', 'FaceAlpha', 0.6)
    end
    text(col_x_rc(1), ry, sum_rc{r,1}, 'FontSize', 7.5, 'Interpreter', 'none')
    text(col_x_rc(2), ry, sum_rc{r,2}, 'FontSize', 7.5, 'Interpreter', 'none', ...
        'FontWeight', 'bold')
end

% Bottom: YZ plane
axes('position', [0.05, 0.13, 0.65, 0.32])
hold on, grid on
title({'YZ Plane — Wishbone Geometry  |  Design Position'; ...
       ['Left corner (lateral accel = +Y)  |  Right wheel = outer  |  Left wheel = inner']}, ...
    'FontWeight', 'bold')
xlabel('y  [m]  (+: left,  -: right)')
ylabel('z  [m]  (+: upward)')

rvab_des   = (par.rvak + par.rvbk) / 2;
rvde_des   = (par.rvdk + par.rvek) / 2;
rvab_l_des = (par_rc_l.rvak + par_rc_l.rvbk) / 2;
rvde_l_des = (par_rc_l.rvdk + par_rc_l.rvek) / 2;

lower_dir   = [par.rvck(2)-rvab_des(2);     par.rvck(3)-rvab_des(3)];
upper_dir   = [par.rvfk(2)-rvde_des(2);     par.rvfk(3)-rvde_des(3)];
lower_dir_l = [par_rc_l.rvck(2)-rvab_l_des(2); par_rc_l.rvck(3)-rvab_l_des(3)];
upper_dir_l = [par_rc_l.rvfk(2)-rvde_l_des(2); par_rc_l.rvfk(3)-rvde_l_des(3)];
lower_dir   = lower_dir   / norm(lower_dir);
upper_dir   = upper_dir   / norm(upper_dir);
lower_dir_l = lower_dir_l / norm(lower_dir_l);
upper_dir_l = upper_dir_l / norm(upper_dir_l);

t_both = linspace(-2, 8, 2);

% Right wishbone
plot([rvab_des(2), par.rvck(2)], [rvab_des(3), par.rvck(3)], ...
    'b-', 'LineWidth', 2.5, 'DisplayName', 'Lower arm - Right')
plot([rvde_des(2), par.rvfk(2)], [rvde_des(3), par.rvfk(3)], ...
    'b--','LineWidth', 2.5, 'DisplayName', 'Upper arm - Right')
plot(par.rvck(2) + t_both*lower_dir(1), par.rvck(3) + t_both*lower_dir(2), ...
    'b:', 'LineWidth', 1, 'HandleVisibility', 'off')
plot(par.rvfk(2) + t_both*upper_dir(1), par.rvfk(3) + t_both*upper_dir(2), ...
    'b:', 'LineWidth', 1, 'HandleVisibility', 'off')

% Left wishbone
plot([rvab_l_des(2), par_rc_l.rvck(2)], [rvab_l_des(3), par_rc_l.rvck(3)], ...
    'r-', 'LineWidth', 2.5, 'DisplayName', 'Lower arm - Left')
plot([rvde_l_des(2), par_rc_l.rvfk(2)], [rvde_l_des(3), par_rc_l.rvfk(3)], ...
    'r--','LineWidth', 2.5, 'DisplayName', 'Upper arm - Left')
plot(par_rc_l.rvck(2) + t_both*lower_dir_l(1), par_rc_l.rvck(3) + t_both*lower_dir_l(2), ...
    'r:', 'LineWidth', 1, 'HandleVisibility', 'off')
plot(par_rc_l.rvfk(2) + t_both*upper_dir_l(1), par_rc_l.rvfk(3) + t_both*upper_dir_l(2), ...
    'r:', 'LineWidth', 1, 'HandleVisibility', 'off')

% Contact patches
CP_r_y = par.rvwk(2);       CP_r_z = par.rvwk(3) - rs;
CP_l_y = par_rc_l.rvwk(2);  CP_l_z = par_rc_l.rvwk(3) - rs;
plot(CP_r_y, CP_r_z, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'CP - Right')
plot(CP_l_y, CP_l_z, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'CP - Left')

% IC positions
P1 = [rvab_des(2); rvab_des(3)];  d1 = lower_dir;
P2 = [rvde_des(2); rvde_des(3)];  d2 = upper_dir;
cross_d = d1(1)*d2(2) - d1(2)*d2(1);
IC_r_des = [NaN; NaN];
if abs(cross_d) > 1e-10
    diff_P   = P2 - P1;
    t_ic     = (diff_P(1)*d2(2) - diff_P(2)*d2(1)) / cross_d;
    IC_r_des = P1 + t_ic * d1;
    plot(IC_r_des(1), IC_r_des(2), 'b+', 'MarkerSize', 14, 'LineWidth', 2.5, ...
        'DisplayName', sprintf('IC Right (%.0f, %.0f mm)', IC_r_des(1)*1000, IC_r_des(2)*1000))
end

P1_l = [rvab_l_des(2); rvab_l_des(3)];  d1_l = lower_dir_l;
P2_l = [rvde_l_des(2); rvde_l_des(3)];  d2_l = upper_dir_l;
cross_l = d1_l(1)*d2_l(2) - d1_l(2)*d2_l(1);
IC_l_des = [NaN; NaN];
if abs(cross_l) > 1e-10
    diff_Pl  = P2_l - P1_l;
    t_ic_l   = (diff_Pl(1)*d2_l(2) - diff_Pl(2)*d2_l(1)) / cross_l;
    IC_l_des = P1_l + t_ic_l * d1_l;
    plot(IC_l_des(1), IC_l_des(2), 'r+', 'MarkerSize', 14, 'LineWidth', 2.5, ...
        'DisplayName', sprintf('IC Left (%.0f, %.0f mm)', IC_l_des(1)*1000, IC_l_des(2)*1000))
end

% RC construction line
if ~any(isnan(IC_r_des))
    t_rc_r = -CP_r_y / (IC_r_des(1) - CP_r_y);
    RC_z_r = CP_r_z + t_rc_r * (IC_r_des(2) - CP_r_z);
    plot([CP_r_y, IC_r_des(1)], [CP_r_z, IC_r_des(2)], ...
        'b-.', 'LineWidth', 1.5, 'HandleVisibility', 'off')
    plot([IC_r_des(1), 0], [IC_r_des(2), RC_z_r], ...
        'b-.', 'LineWidth', 1.5, 'HandleVisibility', 'off')
end

% RC and wheel centre markers
plot(0, rc_height_design, 'g*', 'MarkerSize', 16, 'LineWidth', 2.5, ...
    'DisplayName', sprintf('Design RC: %.1f mm', rc_height_design*1000))
plot(0, rc_height_1g, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('1G RC: %.1f mm', rc_height_1g*1000))
plot(par.rvwk(2),      par.rvwk(3),      'bo', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.7 0.85 1], 'DisplayName', 'Wheel centre - Right')
plot(par_rc_l.rvwk(2), par_rc_l.rvwk(3), 'ro', 'MarkerSize', 8, ...
    'MarkerFaceColor', [1 0.8 0.8],  'DisplayName', 'Wheel centre - Left')

yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Ground')
xline(0, 'k-',  'LineWidth', 0.8, 'HandleVisibility', 'off')
xlim([-0.75, 0.75])
ylim([-0.05, 0.35])
legend('Location', 'eastoutside', 'FontSize', 7.5)

rc_ribbon = {
    ['Front Axle  |  ', num2str(lat_accel_rc), 'G LEFT corner  |  ', ...
 'Lateral accel direction: +Y (leftward)  |  Roll gradient: ', ...
     num2str(roll_gradient_rc), ' deg/G  |  Total roll: ', ...
     sprintf('%.3f', roll_angle_rc_deg), ' deg  |  Right wheel rolls down (jounce)'], ...
    ['Design RC: Z = ', sprintf('%.2f', rc_height_design*1000), ' mm  |  ', ...
     'RC at 1G: Z = ', sprintf('%.2f', rc_height_1g*1000), ' mm  |  ', ...
     'Vertical migration: ', sprintf('%+.2f', rc_vert_migration), ' mm  |  ', ...
     'RC lateral migration: ', sprintf('%+.2f', rc_lat_migration), ' mm'], ...
    'RC from wishbone YZ projections — body roll method (inboard points move with body)', ...
    'Positive RC lateral migration = RC moves toward +Y (left/inner wheel side)'
};
add_sign_box(rc_ribbon, wheel_side)

%% -----------------------------------------------------------------------
%  VALIDATION SUMMARY
%% -----------------------------------------------------------------------
disp('=== KINEMATIC VALIDATION ===')
disp(['Wheelbase (a):      ', num2str(wheelbase*1000, '%.1f'), ' mm'])
disp(['Track width (s):    ', num2str(s*1000, '%.1f'), ' mm'])
disp(['C-factor:          ', num2str(c_factor_in_per_rev), ' in/rev'])
disp(['Max rack travel:   ±', num2str(par.umx*1000, '%.2f'), ' mm'])
disp(['Max steering wheel angle: ±', num2str(max(abs(sw_angle_deg)), '%.1f'), ' deg'])
disp(['Ackermann range:   ', num2str(min(pct_ackermann(valid_ack)), '%.1f'), '% to ', num2str(max(pct_ackermann(valid_ack)), '%.1f'), '%'])