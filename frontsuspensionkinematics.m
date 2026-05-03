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

axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Front-', wheel_side, ' Wheel']; 'Neutral steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)'), ylabel('\Delta\gamma  [deg]  (+: top outboard)')
cj = camb_heave(jounce_idx);  cd_vals = camb_heave(droop_idx);
fill([hj; hj(end); hj(1)], [cj; 0; 0],      [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd; hd(end); hd(1)], [cd_vals; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave, camb_heave, 'r-s', 'LineWidth',2, 'MarkerSize',5, 'DisplayName','Camber change')
plot(heave(n0), camb_heave(n0), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
text(max(heave)*0.1, max(abs(camb_heave))*0.75, 'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(heave)*0.9, max(abs(camb_heave))*0.75, 'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
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

% Left: Suspension camber change vs body roll
axes('position', [0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Suspension Camber Change vs Body Roll'; 'No steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]  (right corner = positive)')
ylabel('\Delta\gamma_{susp}  [deg]  (+: top outboard)')
plot(roll_sweep_deg, camb_outer_roll, 'b-',  'LineWidth', 2, 'DisplayName', 'Outer wheel (right, jounce)')
plot(roll_sweep_deg, camb_inner_roll, 'r--', 'LineWidth', 2, 'DisplayName', 'Inner wheel (left, droop)')
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

% Right: Road-relative camber vs body roll
axes('position', [0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({'Road-Relative Camber vs Body Roll'; 'No steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]  (right corner = positive)')
ylabel('\Delta\gamma_{road}  [deg]  (+: top outboard)')
plot(roll_sweep_deg, camb_outer_road_roll, 'b-',  'LineWidth', 2, 'DisplayName', 'Outer wheel (right)')
plot(roll_sweep_deg, camb_inner_road_roll, 'r--', 'LineWidth', 2, 'DisplayName', 'Inner wheel (left)')
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

% Bottom: Comparison at 1 deg roll
axes('position', [0.20, 0.14, 0.55, 0.33])
hold on, grid on
title({'Suspension vs Road-Relative Camber  |  0 to 1° Body Roll'; ...
       'No steering  |  Outer = right (jounce)  |  Inner = left (droop)'}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
plot(roll_sweep_deg, camb_outer_roll,      'b-',  'LineWidth', 2, 'DisplayName', 'Outer suspension \Delta\gamma')
plot(roll_sweep_deg, camb_outer_road_roll, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Outer road-relative \Delta\gamma')
plot(roll_sweep_deg, camb_inner_roll,      'r-',  'LineWidth', 2, 'DisplayName', 'Inner suspension \Delta\gamma')
plot(roll_sweep_deg, camb_inner_road_roll, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Inner road-relative \Delta\gamma')
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
% Annotate at 1 deg
text(1.01, camb_outer_road_roll(end), sprintf('%+.3f°', camb_outer_road_roll(end)), ...
    'FontSize', 9, 'Color', 'b', 'FontWeight', 'bold')
text(1.01, camb_inner_road_roll(end), sprintf('%+.3f°', camb_inner_road_roll(end)), ...
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

fprintf('\n===== ACKERMANN DIAGNOSTIC =====\n')
fprintf('Min steering threshold: %.1f deg\n', min_steer_threshold)
fprintf('Valid points above threshold: %d / %d\n', sum(~isnan(pct_ackermann)), m)

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
[~, first_v] = find(valid,1);
if ~isempty(first_v)
    text(sw_angle_deg(first_v)*1.05, SR_right(first_v), sprintf('SR_{right} = %.1f', SR_right(first_v)), 'FontSize',8,'Color','blue','FontWeight','bold')
end
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
%  FIGURE 8: Cornering Camber  (was Figure 7)
%% -----------------------------------------------------------------------
lat_accel_G      = 1.4;
roll_gradient    = 0.28;
steer_wheel_deg  = 0;

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
%  VALIDATION SUMMARY
%% -----------------------------------------------------------------------
disp('=== KINEMATIC VALIDATION ===')
disp(['Wheelbase (a):      ', num2str(wheelbase*1000, '%.1f'), ' mm'])
disp(['Track width (s):    ', num2str(s*1000, '%.1f'), ' mm'])
disp(['C-factor:          ', num2str(c_factor_in_per_rev), ' in/rev'])
disp(['Max rack travel:   ±', num2str(par.umx*1000, '%.2f'), ' mm'])
disp(['Max steering wheel angle: ±', num2str(max(abs(sw_angle_deg)), '%.1f'), ' deg'])
disp(['Ackermann range:   ', num2str(min(pct_ackermann(valid_ack)), '%.1f'), '% to ', num2str(max(pct_ackermann(valid_ack)), '%.1f'), '%'])