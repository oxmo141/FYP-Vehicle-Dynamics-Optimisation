%% =========================================================================
%  Suspension Kinematics - Rear Axle (Heave Motion Only, No Steering)
%  =========================================================================
clear; clc; close all
paramR26;

%% =========================================================================
%  >>>  USER PARAMETERS  
%% =========================================================================

par_r.phmx_droop  = 2.8306 / 180 * pi;
par_r.phmx_jounce = 6.007  / 180 * pi;

par_r.rvwk = [-0.66196;  -0.605;       0.2032   ];
par_r.rvak = [-0.585;    -0.255;       0.115484 ];
par_r.rvbk = [-0.470;    -0.284;       0.116    ];
par_r.rvck = [-0.63196;  -0.565;       0.121    ];
par_r.rvdk = [-0.5825;   -0.2465;      0.240343 ];
par_r.rvek = [-0.470;    -0.290;       0.24749  ];
par_r.rvfk = [-0.63196;  -0.5111726;   0.283828 ];
par_r.rvgk = [-0.619;    -0.140;       0.1673839];
par_r.rvhk = [-0.75196;  -0.537826;    0.2032   ];

toe0_r  = 0 / 180 * pi;
camb0_r = 0 / 180 * pi;
rs_r    = car.r_wheel;

n_droop_r  = 10;
n_jounce_r = 15;

wheel_side_r = 'Right';
conv_str_r   = 'ISO 8855';

%% =========================================================================
%  END OF USER PARAMETERS
%% =========================================================================

en0 = [0; 0; 1];

eyrk_r = [toe0_r; 1; -camb0_r];  eyrk_r = eyrk_r / norm(eyrk_r);
exk_r  = cross(eyrk_r, en0);     exk_r  = exk_r  / norm(exk_r);
eyk_r  = cross(en0,    exk_r);
ezk_r  = cross(exk_r,  eyrk_r);
rwpk_r = -rs_r * ezk_r;

rcfk_r = par_r.rvfk - par_r.rvck;
ecfk_r = rcfk_r / norm(rcfk_r);

si_r = atan2(-ecfk_r(2), ecfk_r(3));
nu_r = atan2(ecfk_r(1), ecfk_r(3));

disp(['[REAR] sigma (KPI)   = ', num2str(si_r * 180/pi, '%.3f'), ' deg'])
disp(['[REAR] nu   (caster) = ', num2str(nu_r * 180/pi, '%.3f'), ' deg'])

rcpk_r = par_r.rvwk + rwpk_r - par_r.rvck;
rsck_r = -(en0.' * rcpk_r) / (en0.' * ecfk_r) * ecfk_r;
co_r   = -exk_r' * (rsck_r + rcpk_r);
sr_r   =  eyk_r' * (rsck_r + rcpk_r);
disp(['[REAR] caster offset = ', num2str(co_r, '%.4f')])
disp(['[REAR] scrub radius  = ', num2str(sr_r, '%.4f')])

rvpk_r = par_r.rvwk + rwpk_r;

sign_block_r = {
    ['Analyzed Wheel: Rear-', wheel_side_r, ' (negative Y)'], ...
    ['Opposite Wheel: Rear-', rear_conditional_string(wheel_side_r)], ...
    ['Coord. system: ', conv_str_r, ' (X fwd, Y left, Z up)'], ...
    'Toe  (+): toe-in  (+Z rotation)', ...
    'Camber (+): top outboard  (+X rotation)', ...
    'Phi  (+): jounce/compression  |  Phi (-): droop/rebound', ...
    'Heave motion only — no steering input'
};

n_r = n_droop_r + n_jounce_r - 1;

phi_droop_r  = linspace(-par_r.phmx_droop,  0, n_droop_r);
phi_jounce_r = linspace(0, par_r.phmx_jounce, n_jounce_r);
phi_r        = [phi_droop_r(1:end-1), phi_jounce_r];

[~, n0_r] = min(abs(phi_r));

disp(' ')
disp('Rear suspension travel discretization:')
disp(['  Droop:  ', num2str(n_droop_r), ' steps,  ', ...
      num2str(-par_r.phmx_droop * 180/pi, '%.4f'), ' deg'])
disp(['  Jounce: ', num2str(n_jounce_r), ' steps, +', ...
      num2str(par_r.phmx_jounce * 180/pi, '%.4f'), ' deg'])
disp(['  Total:  ', num2str(n_r), ' steps'])
disp(['  Design position index n0_r = ', num2str(n0_r), ...
      '  (phi = ', num2str(phi_r(n0_r)*180/pi, '%.3f'), ' deg)'])

par_l = par_r;
fields_mirror = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk','rvgk','rvhk'};
for f = 1:length(fields_mirror)
    par_l.(fields_mirror{f})(2) = -par_r.(fields_mirror{f})(2);
end

fprintf('\n=== TOE LINK SYMMETRY CHECK ===\n')
fprintf('Toe link length - Right: %.6f m\n', norm(par_r.rvhk - par_r.rvgk))
fprintf('Toe link length - Left:  %.6f m\n', norm(par_l.rvhk - par_l.rvgk))
fprintf('Difference:              %.2e m\n', ...
    norm(par_r.rvhk - par_r.rvgk) - norm(par_l.rvhk - par_l.rvgk))

%% -----------------------------------------------------------------------
%  RIGHT WHEEL SWEEP
%% -----------------------------------------------------------------------
toe_right  = zeros(n_r, 1);
camb_right = zeros(n_r, 1);
xw_r  = zeros(n_r, 1);
yw_r  = zeros(n_r, 1);
zw_r  = zeros(n_r, 1);
xp_r  = zeros(n_r, 1);
yp_r  = zeros(n_r, 1);
zp_r  = zeros(n_r, 1);

toe_prev_r = 0;

for i = 1:n_r
    [avw_r, rvwv_r, toe_right(i), camb_right(i)] = ...
        fun_rear_kin(phi_r(i), par_r, toe_prev_r);
    toe_prev_r = toe_right(i);
    rvpv_r   = rvwv_r + avw_r * rwpk_r;
    xw_r(i)  = rvwv_r(1);
    yw_r(i)  = rvwv_r(2);
    zw_r(i)  = rvwv_r(3);
    xp_r(i)  = rvpv_r(1);
    yp_r(i)  = rvpv_r(2);
    zp_r(i)  = rvpv_r(3);
end

%% -----------------------------------------------------------------------
%  LEFT WHEEL SWEEP
%% -----------------------------------------------------------------------
toe_left  = zeros(n_r, 1);
camb_left = zeros(n_r, 1);

toe_prev_l = 0;

for i = 1:n_r
    [~, ~, toe_raw_l, camb_left(i)] = ...
        fun_rear_kin(phi_r(i), par_l, toe_prev_l);
    toe_left(i) = -toe_raw_l;
    toe_prev_l  =  toe_raw_l;
end

%% -----------------------------------------------------------------------
%  NEUTRAL POSITION SUMMARY
%% -----------------------------------------------------------------------
fprintf('\n%s\n', repmat('=',1,70))
fprintf('REAR AXLE - NEUTRAL POSITION (phi=0)\n')
fprintf('%s\n', repmat('=',1,70))
fprintf('Toe   - Right: %+.4f deg  |  Left: %+.4f deg\n', ...
    toe_right(n0_r)*180/pi, toe_left(n0_r)*180/pi)
fprintf('Camber - Right: %+.4f deg  |  Left: %+.4f deg\n', ...
    camb_right(n0_r)*180/pi, camb_left(n0_r)*180/pi)
fprintf('%s\n\n', repmat('=',1,70))

%% -----------------------------------------------------------------------
%  HEAVE EXCURSION
%% -----------------------------------------------------------------------
z_design_r   = zw_r(n0_r);
heave_r      = (zw_r - z_design_r) * 1000;

toe_right_chg  = (toe_right  - toe_right(n0_r))  * 180/pi;
camb_right_chg = (camb_right - camb_right(n0_r)) * 180/pi;
toe_left_chg   = (toe_left   - toe_left(n0_r))   * 180/pi;
camb_left_chg  = (camb_left  - camb_left(n0_r))  * 180/pi;

toe_design_R  = toe_right(n0_r)  * 180/pi;
camb_design_R = camb_right(n0_r) * 180/pi;
toe_design_L  = toe_left(n0_r)   * 180/pi;
camb_design_L = camb_left(n0_r)  * 180/pi;

jounce_idx_r = find(heave_r >= 0);
droop_idx_r  = find(heave_r <= 0);

hj_r = heave_r(jounce_idx_r);
hd_r = heave_r(droop_idx_r);

%% -----------------------------------------------------------------------
%  FIGURE 1: Wheel Centre Paths + Toe & Camber vs Phi
%% -----------------------------------------------------------------------
figure('Name', ['Figure 1 - Rear Wheel Centre Paths  |  Analyzed: ', wheel_side_r])

axes('position',[0.05, 0.12, 0.18, 0.78])
hold on, grid on
title({'Longitudinal Plane (XZ)'; ['Rear-', wheel_side_r]}, 'FontWeight','bold')
xlabel('x  [m]  (+ forward)')
ylabel('z  [m]  (+ upward)')
plot(xw_r, zw_r, 'b-',  'LineWidth',1.5, 'DisplayName','W centre')
plot(xp_r, zp_r, 'b--', 'LineWidth',1.5, 'DisplayName','Contact P')
plot(par_r.rvwk(1), par_r.rvwk(3), 'ok', 'MarkerFaceColor','k', 'MarkerSize',6, 'DisplayName','W design')
plot(rvpk_r(1), rvpk_r(3), 'o', 'Color','cyan', 'MarkerFaceColor','cyan','MarkerSize',6, 'DisplayName','P design')
x_pad = max(0.002, 0.5*(max(xw_r)-min(xw_r)));
z_pad = max(0.005, 0.1*(max(zw_r)-min(zw_r)));
xlim([min(xw_r)-x_pad, max(xw_r)+x_pad])
ylim([min(zw_r)-z_pad, max(zw_r)+z_pad])
legend('Location','best','FontSize',8)

axes('position',[0.28, 0.12, 0.18, 0.78])
hold on, grid on
title({'Lateral Plane (YZ)'; ['Rear-', wheel_side_r]}, 'FontWeight','bold')
xlabel({'y  [m]'; '(+left, -Y=right)'})
ylabel('z  [m]  (+ upward)')
plot(yw_r, zw_r, 'r-',  'LineWidth',1.5, 'DisplayName','W centre')
plot(yp_r, zp_r, 'r--', 'LineWidth',1.5, 'DisplayName','Contact P')
plot(par_r.rvwk(2), par_r.rvwk(3), 'ok', 'MarkerFaceColor','k', 'MarkerSize',6, 'DisplayName','W design')
plot(rvpk_r(2), rvpk_r(3), 'o', 'Color','cyan', 'MarkerFaceColor','cyan','MarkerSize',6, 'DisplayName','P design')
y_pad = max(0.002, 0.5*(max(yw_r)-min(yw_r)));
z_pad = max(0.005, 0.1*(max(zw_r)-min(zw_r)));
xlim([min(yw_r)-y_pad, max(yw_r)+y_pad])
ylim([min(zw_r)-z_pad, max(zw_r)+z_pad])
legend('Location','best','FontSize',8)

axes('position',[0.55, 0.55, 0.42, 0.35])
hold on, grid on
title({'Toe Angle vs Lower Arm Rotation'; '(+) = toe-in'}, 'FontWeight','bold')
xlabel('\phi  [deg]  (+: jounce,  -: droop)')
ylabel('Toe  [deg]  (+: toe-in)')
plot(phi_r*180/pi, toe_right*180/pi, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Right (', wheel_side_r, ')'])
plot(phi_r*180/pi, toe_left*180/pi,  'r--s','LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Left (', rear_conditional_string(wheel_side_r), ')'])
plot(phi_r(n0_r)*180/pi, toe_right(n0_r)*180/pi, 'g*', 'MarkerSize',14, 'LineWidth',2,'HandleVisibility','off')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
legend('Location','best','FontSize',8)

axes('position',[0.55, 0.12, 0.42, 0.35])
hold on, grid on
title({'Camber Angle vs Lower Arm Rotation'; '(+) = top outboard'}, 'FontWeight','bold')
xlabel('\phi  [deg]  (+: jounce,  -: droop)')
ylabel('Camber  [deg]  (+: top outboard)')
plot(phi_r*180/pi, camb_right*180/pi, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Right (', wheel_side_r, ')'])
plot(phi_r*180/pi, camb_left*180/pi,  'r--s','LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Left (', rear_conditional_string(wheel_side_r), ')'])
plot(phi_r(n0_r)*180/pi, camb_right(n0_r)*180/pi, 'g*', 'MarkerSize',14, 'LineWidth',2,'HandleVisibility','off')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
legend('Location','best','FontSize',8)

add_sign_box_rear(sign_block_r)

%% -----------------------------------------------------------------------
%  FIGURE 2: Toe & Camber During Heave - Both Wheels
%% -----------------------------------------------------------------------
figure('Name', 'Figure 2 - Rear Heave Response  |  Both Wheels')

heave_r_m          = (zw_r - z_design_r);
p_camb_R           = polyfit(heave_r_m, camb_right_chg, 1);
ride_camber_rate_R = p_camb_R(1);
p_camb_L           = polyfit(heave_r_m, camb_left_chg,  1);
ride_camber_rate_L = p_camb_L(1);

axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({['Toe Change vs Heave  |  Rear-', wheel_side_r]}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
tj_R = toe_right_chg(jounce_idx_r);
td_R = toe_right_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [tj_R; 0; 0], [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [td_R; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, toe_right_chg, 'b-o', 'LineWidth',2, 'MarkerSize',5, 'DisplayName',['Right (', wheel_side_r, ')'])
plot(heave_r(n0_r), toe_right_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
if max(abs(toe_right_chg)) > 0
    text(max(heave_r)*0.1, max(abs(toe_right_chg))*0.75, 'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
    text(min(heave_r)*0.9, max(abs(toe_right_chg))*0.75, 'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
end
legend('Location','southwest','FontSize',9)

axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Rear-', wheel_side_r]; ...
       ['Ride camber rate: ', sprintf('%.2f', ride_camber_rate_R), ' deg/m']}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
cj_R = camb_right_chg(jounce_idx_r);
cd_R = camb_right_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [cj_R; 0; 0], [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [cd_R; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, camb_right_chg, 'r-s', 'LineWidth',2, 'MarkerSize',5, 'DisplayName',['Right (', wheel_side_r, ')'])
plot(heave_r(n0_r), camb_right_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
heave_fit_mm_R = linspace(min(heave_r), max(heave_r), 100);
plot(heave_fit_mm_R, polyval(p_camb_R, heave_fit_mm_R/1000), 'k--', 'LineWidth', 1, ...
    'DisplayName', sprintf('Linear fit: %.2f deg/m', ride_camber_rate_R))
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
if max(abs(camb_right_chg)) > 0
    text(max(heave_r)*0.1, max(abs(camb_right_chg))*0.75, 'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
    text(min(heave_r)*0.9, max(abs(camb_right_chg))*0.75, 'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
end
legend('Location','southwest','FontSize',9)

axes('position',[0.06, 0.14, 0.38, 0.33])
hold on, grid on
title({['Toe Change vs Heave  |  Rear-', rear_conditional_string(wheel_side_r)]}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
tj_L = toe_left_chg(jounce_idx_r);
td_L = toe_left_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [tj_L; 0; 0], [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [td_L; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, toe_left_chg, 'b-o', 'LineWidth',2, 'MarkerSize',5, 'DisplayName',['Left (', rear_conditional_string(wheel_side_r), ')'])
plot(heave_r(n0_r), toe_left_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
if max(abs(toe_left_chg)) > 0
    text(max(heave_r)*0.1, max(abs(toe_left_chg))*0.75, 'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
    text(min(heave_r)*0.9, max(abs(toe_left_chg))*0.75, 'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
end
legend('Location','southwest','FontSize',9)

axes('position',[0.56, 0.14, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Rear-', rear_conditional_string(wheel_side_r)]; ...
       ['Ride camber rate: ', sprintf('%.2f', ride_camber_rate_L), ' deg/m']}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
cj_L = camb_left_chg(jounce_idx_r);
cd_L = camb_left_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [cj_L; 0; 0], [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [cd_L; 0; 0], [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, camb_left_chg, 'r-s', 'LineWidth',2, 'MarkerSize',5, 'DisplayName',['Left (', rear_conditional_string(wheel_side_r), ')'])
plot(heave_r(n0_r), camb_left_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
heave_fit_mm_L = linspace(min(heave_r), max(heave_r), 100);
plot(heave_fit_mm_L, polyval(p_camb_L, heave_fit_mm_L/1000), 'k--', 'LineWidth', 1, ...
    'DisplayName', sprintf('Linear fit: %.2f deg/m', ride_camber_rate_L))
yline(0,'k:','HandleVisibility','off'), xline(0,'k:','HandleVisibility','off')
if max(abs(camb_left_chg)) > 0
    text(max(heave_r)*0.1, max(abs(camb_left_chg))*0.75, 'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
    text(min(heave_r)*0.9, max(abs(camb_left_chg))*0.75, 'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
end
legend('Location','southwest','FontSize',9)

add_sign_box_rear(sign_block_r)

%% -----------------------------------------------------------------------
%  FIGURE 3: Camber vs Body Roll (0 to 1 deg)
%% -----------------------------------------------------------------------
n_roll_r          = 100;
roll_sweep_r_deg  = linspace(0, 1, n_roll_r);
camb_outer_roll_r = zeros(1, n_roll_r);
camb_inner_roll_r = zeros(1, n_roll_r);

for k = 1:n_roll_r
    phi_o_r =  (roll_sweep_r_deg(k) / 2) / 180 * pi;
    phi_i_r = -(roll_sweep_r_deg(k) / 2) / 180 * pi;
    [~, ~, ~, camb_o_raw]      = fun_rear_kin(phi_o_r, par_r, 0);
    camb_outer_roll_r(k)       = camb_o_raw * 180/pi - camb_design_R;
    [~, ~, ~, camb_i_raw]      = fun_rear_kin(phi_i_r, par_l, 0);
    camb_inner_roll_r(k)       = camb_i_raw * 180/pi - camb_design_L;
end

camb_outer_road_roll_r = camb_outer_roll_r - roll_sweep_r_deg / 2;
camb_inner_road_roll_r = camb_inner_roll_r + roll_sweep_r_deg / 2;

p_outer_susp_r = polyfit(roll_sweep_r_deg, camb_outer_roll_r,      1);
p_inner_susp_r = polyfit(roll_sweep_r_deg, camb_inner_roll_r,      1);
p_outer_road_r = polyfit(roll_sweep_r_deg, camb_outer_road_roll_r, 1);
p_inner_road_r = polyfit(roll_sweep_r_deg, camb_inner_road_roll_r, 1);

rate_outer_susp_r = p_outer_susp_r(1);
rate_inner_susp_r = p_inner_susp_r(1);
rate_outer_road_r = p_outer_road_r(1);
rate_inner_road_r = p_inner_road_r(1);

fprintf('\n===== REAR AXLE CAMBER RATES =====\n')
fprintf('Ride camber rate - Right: %.4f deg/m\n', ride_camber_rate_R)
fprintf('Ride camber rate - Left:  %.4f deg/m\n', ride_camber_rate_L)
fprintf('\nRoll camber rate (suspension only):\n')
fprintf('  Outer (right, jounce):  %.4f deg/deg\n', rate_outer_susp_r)
fprintf('  Inner (left,  droop):   %.4f deg/deg\n', rate_inner_susp_r)
fprintf('\nRoll camber rate (road-relative):\n')
fprintf('  Outer (right):          %.4f deg/deg\n', rate_outer_road_r)
fprintf('  Inner (left):           %.4f deg/deg\n', rate_inner_road_r)

figure('Name', 'Figure 3 - Rear Camber vs Body Roll  |  0 to 1 deg  |  No Steering')

axes('position', [0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Suspension Camber Change vs Body Roll  |  Rear Axle'; ...
       ['Outer: ', sprintf('%.4f', rate_outer_susp_r), ' deg/deg  |  ', ...
        'Inner: ', sprintf('%.4f', rate_inner_susp_r), ' deg/deg']}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]  (right corner = positive)')
ylabel('\Delta\gamma_{susp}  [deg]  (+: top outboard)')
plot(roll_sweep_r_deg, camb_outer_roll_r, 'b-',  'LineWidth', 2, 'DisplayName', ...
    ['Outer - Right (jounce)  ', sprintf('%.4f deg/deg', rate_outer_susp_r)])
plot(roll_sweep_r_deg, camb_inner_roll_r, 'r--', 'LineWidth', 2, 'DisplayName', ...
    ['Inner - Left  (droop)   ', sprintf('%.4f deg/deg', rate_inner_susp_r)])
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

axes('position', [0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({'Road-Relative Camber vs Body Roll  |  Rear Axle'; ...
       ['Outer: ', sprintf('%.4f', rate_outer_road_r), ' deg/deg  |  ', ...
        'Inner: ', sprintf('%.4f', rate_inner_road_r), ' deg/deg']}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]  (right corner = positive)')
ylabel('\Delta\gamma_{road}  [deg]  (+: top outboard)')
plot(roll_sweep_r_deg, camb_outer_road_roll_r, 'b-',  'LineWidth', 2, 'DisplayName', ...
    ['Outer - Right  ', sprintf('%.4f deg/deg', rate_outer_road_r)])
plot(roll_sweep_r_deg, camb_inner_road_roll_r, 'r--', 'LineWidth', 2, 'DisplayName', ...
    ['Inner - Left   ', sprintf('%.4f deg/deg', rate_inner_road_r)])
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 9)

axes('position', [0.20, 0.14, 0.55, 0.33])
hold on, grid on
title({'Suspension vs Road-Relative Camber  |  0 to 1 deg Body Roll  |  Rear Axle'; ...
       'No steering  |  Outer = right (jounce)  |  Inner = left (droop)'}, 'FontWeight','bold')
xlabel('Body roll angle  [deg]')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
plot(roll_sweep_r_deg, camb_outer_roll_r,      'b-',  'LineWidth', 2,   'DisplayName', 'Outer suspension \Delta\gamma')
plot(roll_sweep_r_deg, camb_outer_road_roll_r, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Outer road-relative \Delta\gamma')
plot(roll_sweep_r_deg, camb_inner_roll_r,      'r-',  'LineWidth', 2,   'DisplayName', 'Inner suspension \Delta\gamma')
plot(roll_sweep_r_deg, camb_inner_road_roll_r, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Inner road-relative \Delta\gamma')
yline(0, 'k:', 'HandleVisibility', 'off')
xline(0, 'k:', 'HandleVisibility', 'off')
text(1.01, camb_outer_road_roll_r(end), ...
    sprintf('%+.3f° (%+.4f deg/deg)', camb_outer_road_roll_r(end), rate_outer_road_r), ...
    'FontSize', 9, 'Color', 'b', 'FontWeight', 'bold')
text(1.01, camb_inner_road_roll_r(end), ...
    sprintf('%+.3f° (%+.4f deg/deg)', camb_inner_road_roll_r(end), rate_inner_road_r), ...
    'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold')
legend('Location', 'best', 'FontSize', 9)

roll_sign_block_r = {
    'Rear Axle  |  No steering input', ...
    'Body roll split 50/50 — each wheel sees half total roll as phi', ...
    'Suspension camber: change due to suspension geometry only', ...
    'Road-relative camber: suspension change + body roll tilt contribution', ...
    'Roll camber rate (+): camber gain helps tyre lean into corner  |  (-): camber loss'
};
add_sign_box_rear(roll_sign_block_r)

%% -----------------------------------------------------------------------
%  ROLL CENTRE ESTIMATION — REAR AXLE
%% -----------------------------------------------------------------------
roll_gradient_rc_r  = 0.28;
lat_accel_rc_r      = 1.0;
roll_angle_rc_deg_r = lat_accel_rc_r * roll_gradient_rc_r;

phi_rc_outer_r =  (roll_angle_rc_deg_r / 2) / 180 * pi;
phi_rc_inner_r = -(roll_angle_rc_deg_r / 2) / 180 * pi;

par_rc_l_r = par_r;
fields_rc_r = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk'};
for f = 1:length(fields_rc_r)
    par_rc_l_r.(fields_rc_r{f})(2) = -par_r.(fields_rc_r{f})(2);
end

% Build phi sweep with exact zero included
n_rc_r_half    = 100;
phi_rc_neg_r   = linspace(-par_r.phmx_droop,   0, n_rc_r_half);
phi_rc_pos_r   = linspace(0, par_r.phmx_jounce, n_rc_r_half);
phi_rc_sweep_r = [phi_rc_neg_r(1:end-1), phi_rc_pos_r];
n_rc_r         = length(phi_rc_sweep_r);

rc_height_r    = zeros(1, n_rc_r);
rc_y_r         = zeros(1, n_rc_r);
IC_r_y_sweep_r = zeros(1, n_rc_r);
IC_r_z_sweep_r = zeros(1, n_rc_r);
CP_r_y_sweep_r = zeros(1, n_rc_r);
CP_r_z_sweep_r = zeros(1, n_rc_r);
yw_rc_rr       = zeros(1, n_rc_r);
zw_rc_rr       = zeros(1, n_rc_r);

% Static IC from hardpoints
rvab_rr  = (par_r.rvak + par_r.rvbk) / 2;
rvde_rr  = (par_r.rvdk + par_r.rvek) / 2;
d1_rr    = [par_r.rvck(2)-rvab_rr(2); par_r.rvck(3)-rvab_rr(3)];
d2_rr    = [par_r.rvfk(2)-rvde_rr(2); par_r.rvfk(3)-rvde_rr(3)];
P1_rr    = [rvab_rr(2); rvab_rr(3)];
P2_rr    = [rvde_rr(2); rvde_rr(3)];
cross_rr = d1_rr(1)*d2_rr(2) - d1_rr(2)*d2_rr(1);
diff_rr  = P2_rr - P1_rr;
t_rr     = (diff_rr(1)*d2_rr(2) - diff_rr(2)*d2_rr(1)) / cross_rr;
IC_r_static_r = P1_rr + t_rr * d1_rr;

rvab_lr  = (par_rc_l_r.rvak + par_rc_l_r.rvbk) / 2;
rvde_lr  = (par_rc_l_r.rvdk + par_rc_l_r.rvek) / 2;
d1_lr    = [par_rc_l_r.rvck(2)-rvab_lr(2); par_rc_l_r.rvck(3)-rvab_lr(3)];
d2_lr    = [par_rc_l_r.rvfk(2)-rvde_lr(2); par_rc_l_r.rvfk(3)-rvde_lr(3)];
P1_lr    = [rvab_lr(2); rvab_lr(3)];
P2_lr    = [rvde_lr(2); rvde_lr(3)];
cross_lr = d1_lr(1)*d2_lr(2) - d1_lr(2)*d2_lr(1);
diff_lr  = P2_lr - P1_lr;
t_lr     = (diff_lr(1)*d2_lr(2) - diff_lr(2)*d2_lr(1)) / cross_lr;
IC_l_static_r = P1_lr + t_lr * d1_lr;

fprintf('\nRear Static IC - Right: Y = %.2f mm,  Z = %.2f mm\n', ...
    IC_r_static_r(1)*1000, IC_r_static_r(2)*1000)
fprintf('Rear Static IC - Left:  Y = %.2f mm,  Z = %.2f mm\n', ...
    IC_l_static_r(1)*1000, IC_l_static_r(2)*1000)

% Main sweep loop — body roll method with cumulative toe seeds
toe_seed_r = 0;
toe_seed_l = 0;

for k = 1:n_rc_r
    phi_k_r = phi_rc_sweep_r(k);
    alpha_r = phi_k_r;

    R_roll_r = [cos(alpha_r), -sin(alpha_r); ...
                sin(alpha_r),  cos(alpha_r)];

    % Right wheel — inboard points rotated with body
    rvab_r_yz = R_roll_r * [rvab_rr(2); rvab_rr(3)];
    rvde_r_yz = R_roll_r * [rvde_rr(2); rvde_rr(3)];

    [~, rvwv_rr_k, toe_seed_r, ~] = fun_rear_kin(phi_k_r, par_r, toe_seed_r);

    dZ_rr = rvwv_rr_k(3) - par_r.rvwk(3);
    dY_rr = rvwv_rr_k(2) - par_r.rvwk(2);

    rvc_r_yz = [par_r.rvck(2) + dY_rr; par_r.rvck(3) + dZ_rr];
    rvf_r_yz = [par_r.rvfk(2) + dY_rr; par_r.rvfk(3) + dZ_rr];

    d1_kr    = rvc_r_yz - rvab_r_yz;
    d2_kr    = rvf_r_yz - rvde_r_yz;
    cross_kr = d1_kr(1)*d2_kr(2) - d1_kr(2)*d2_kr(1);

    if abs(cross_kr) < 1e-10
        IC_r_k = IC_r_static_r;
    else
        diff_kr = rvde_r_yz - rvab_r_yz;
        t_kr    = (diff_kr(1)*d2_kr(2) - diff_kr(2)*d2_kr(1)) / cross_kr;
        IC_r_k  = rvab_r_yz + t_kr * d1_kr;
    end

    CP_r_k = [rvwv_rr_k(2); rvwv_rr_k(3) - rs_r];

    % Left wheel — inboard points rotated with body
    rvab_l_yz_r = R_roll_r * [rvab_lr(2); rvab_lr(3)];
    rvde_l_yz_r = R_roll_r * [rvde_lr(2); rvde_lr(3)];

    [~, rvwv_lr_k, toe_raw_l_k, ~] = fun_rear_kin(-phi_k_r, par_rc_l_r, toe_seed_l);
    toe_seed_l = toe_raw_l_k;

    dZ_lr = rvwv_lr_k(3) - par_rc_l_r.rvwk(3);
    dY_lr = rvwv_lr_k(2) - par_rc_l_r.rvwk(2);

    rvc_l_yz_r = [par_rc_l_r.rvck(2) + dY_lr; par_rc_l_r.rvck(3) + dZ_lr];
    rvf_l_yz_r = [par_rc_l_r.rvfk(2) + dY_lr; par_rc_l_r.rvfk(3) + dZ_lr];

    d1_kl_r    = rvc_l_yz_r - rvab_l_yz_r;
    d2_kl_r    = rvf_l_yz_r - rvde_l_yz_r;
    cross_kl_r = d1_kl_r(1)*d2_kl_r(2) - d1_kl_r(2)*d2_kl_r(1);

    if abs(cross_kl_r) < 1e-10
        IC_l_k = IC_l_static_r;
    else
        diff_kl_r = rvde_l_yz_r - rvab_l_yz_r;
        t_kl_r    = (diff_kl_r(1)*d2_kl_r(2) - diff_kl_r(2)*d2_kl_r(1)) / cross_kl_r;
        IC_l_k    = rvab_l_yz_r + t_kl_r * d1_kl_r;
    end

    CP_l_k = [rvwv_lr_k(2); rvwv_lr_k(3) - rs_r];

    CP_r_y_sweep_r(k) = CP_r_k(1);
    CP_r_z_sweep_r(k) = CP_r_k(2);
    yw_rc_rr(k)       = rvwv_rr_k(2);
    zw_rc_rr(k)       = rvwv_rr_k(3);
    IC_r_y_sweep_r(k) = IC_r_k(1);
    IC_r_z_sweep_r(k) = IC_r_k(2);

    dir_r_k = IC_r_k - CP_r_k;
    dir_l_k = IC_l_k - CP_l_k;

    A_k     = [dir_r_k(1), -dir_l_k(1); dir_r_k(2), -dir_l_k(2)];
    b_k     = CP_l_k - CP_r_k;
    det_A_k = A_k(1,1)*A_k(2,2) - A_k(1,2)*A_k(2,1);

    if abs(det_A_k) < 1e-12
        rc_y_r(k)      = 0;
        rc_height_r(k) = (CP_r_k(2) + CP_l_k(2)) / 2;
        continue
    end

    t_sol_k        = (b_k(1)*A_k(2,2) - b_k(2)*A_k(1,2)) / det_A_k;
    RC_pt_k        = CP_r_k + t_sol_k * dir_r_k;
    rc_y_r(k)      = RC_pt_k(1);
    rc_height_r(k) = RC_pt_k(2);
end

% Key indices and values
[~, idx_des_rc_r] = min(abs(phi_rc_sweep_r));
[~, idx_1g_r]     = min(abs(phi_rc_sweep_r - phi_rc_outer_r));

fprintf('\nAfter fix:\n')
fprintf('idx_des_rc_r = %d,  phi = %.6f deg\n', idx_des_rc_r, phi_rc_sweep_r(idx_des_rc_r)*180/pi)
fprintf('idx_1g_r     = %d,  phi = %.6f deg\n', idx_1g_r,     phi_rc_sweep_r(idx_1g_r)*180/pi)
fprintf('Separation:  %.6f deg\n', ...
    (phi_rc_sweep_r(idx_1g_r) - phi_rc_sweep_r(idx_des_rc_r))*180/pi)

rc_height_design_r  = rc_height_r(idx_des_rc_r);
rc_height_1g_r      = rc_height_r(idx_1g_r);
rc_vert_migration_r = (rc_height_1g_r - rc_height_design_r) * 1000;

rc_y_design_r       = rc_y_r(idx_des_rc_r);
rc_y_1g_r           = rc_y_r(idx_1g_r);
rc_lat_migration_r  = (rc_y_1g_r - rc_y_design_r) * 1000;

IC_r_y_design_r = IC_r_y_sweep_r(idx_des_rc_r);
IC_r_z_design_r = IC_r_z_sweep_r(idx_des_rc_r);
IC_r_y_1g_r     = IC_r_y_sweep_r(idx_1g_r);
IC_r_z_1g_r     = IC_r_z_sweep_r(idx_1g_r);
CP_r_y_design_r = CP_r_y_sweep_r(idx_des_rc_r);
CP_r_y_1g_r     = CP_r_y_sweep_r(idx_1g_r);

IC_y_migration_r = (IC_r_y_1g_r - IC_r_y_design_r) * 1000;
CP_y_migration_r = (CP_r_y_1g_r - CP_r_y_design_r) * 1000;

heave_rc_mm_r = interp1(phi_r, heave_r, phi_rc_sweep_r, 'linear', 'extrap');

fprintf('\n===== REAR ROLL CENTRE ESTIMATION =====\n')
fprintf('Design RC height:         %.2f mm\n',  rc_height_design_r*1000)
fprintf('RC height at 1G:          %.2f mm\n',  rc_height_1g_r*1000)
fprintf('RC vertical migration:    %+.2f mm\n', rc_vert_migration_r)
fprintf('RC lateral migration:     %+.2f mm\n', rc_lat_migration_r)
fprintf('IC lateral migration:     %+.2f mm\n', IC_y_migration_r)
fprintf('CP lateral migration:     %+.2f mm\n', CP_y_migration_r)

fprintf('\n===== REAR RC LOOP DIAGNOSTIC =====\n')
fprintf('CP_r_y at design (k=%d): %.4f mm\n', idx_des_rc_r, CP_r_y_sweep_r(idx_des_rc_r)*1000)
fprintf('CP_r_y at 1G    (k=%d): %.4f mm\n', idx_1g_r,     CP_r_y_sweep_r(idx_1g_r)*1000)
fprintf('dCP_r_y:                 %.4f mm\n', (CP_r_y_sweep_r(idx_1g_r)-CP_r_y_sweep_r(idx_des_rc_r))*1000)
fprintf('IC_r_y at design:        %.4f mm\n', IC_r_y_sweep_r(idx_des_rc_r)*1000)
fprintf('IC_r_y at 1G:            %.4f mm\n', IC_r_y_sweep_r(idx_1g_r)*1000)
fprintf('RC_y at design:          %.4f mm\n', rc_y_r(idx_des_rc_r)*1000)
fprintf('RC_y at 1G:              %.4f mm\n', rc_y_r(idx_1g_r)*1000)
fprintf('RC_z at design:          %.4f mm\n', rc_height_r(idx_des_rc_r)*1000)
fprintf('RC_z at 1G:              %.4f mm\n', rc_height_r(idx_1g_r)*1000)

%% -----------------------------------------------------------------------
%  FIGURE 4: Roll Centre Migration — Rear Axle
%% -----------------------------------------------------------------------
figure('Name', 'Figure 4 - Roll Centre Migration  |  Rear Axle')

axes('position', [0.05, 0.57, 0.27, 0.35])
hold on, grid on
title({'RC Height vs Heave  |  Rear Axle'; ...
       ['Design: ', sprintf('%.1f', rc_height_design_r*1000), ' mm  |  ', ...
        '1G: ',     sprintf('%.1f', rc_height_1g_r*1000),    ' mm  |  ', ...
        'Migration: ', sprintf('%+.1f', rc_vert_migration_r), ' mm']}, ...
    'FontWeight', 'bold')
xlabel('Wheel centre heave  [mm]  (+: jounce)')
ylabel('RC height  [mm]  (+ above ground)')

ylims_rc_r = [min(rc_height_r)*1000*0.85, max(rc_height_r)*1000*1.1];
fill([0, max(heave_rc_mm_r), max(heave_rc_mm_r), 0], ...
     [ylims_rc_r(1), ylims_rc_r(1), ylims_rc_r(2), ylims_rc_r(2)], ...
     [1 0.9 0.9], 'EdgeColor','none','FaceAlpha',0.3,'HandleVisibility','off')
fill([min(heave_rc_mm_r), 0, 0, min(heave_rc_mm_r)], ...
     [ylims_rc_r(1), ylims_rc_r(1), ylims_rc_r(2), ylims_rc_r(2)], ...
     [0.9 0.9 1], 'EdgeColor','none','FaceAlpha',0.3,'HandleVisibility','off')
plot(heave_rc_mm_r, rc_height_r*1000, 'b-', 'LineWidth', 2.5, 'DisplayName', 'RC height')
plot(heave_rc_mm_r(idx_des_rc_r), rc_height_design_r*1000, 'g*', ...
    'MarkerSize', 14, 'LineWidth', 2, ...
    'DisplayName', sprintf('Design: %.1f mm', rc_height_design_r*1000))
plot(heave_rc_mm_r(idx_1g_r), rc_height_1g_r*1000, 'r^', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('At 1G: %.1f mm', rc_height_1g_r*1000))
yline(0, 'k--', 'LineWidth', 1, 'DisplayName', 'Ground')
xline(0, 'k:',  'LineWidth', 1, 'HandleVisibility', 'off')
text(max(heave_rc_mm_r)*0.5,  ylims_rc_r(1)+(ylims_rc_r(2)-ylims_rc_r(1))*0.06, ...
    'JOUNCE', 'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(heave_rc_mm_r)*0.55, ylims_rc_r(1)+(ylims_rc_r(2)-ylims_rc_r(1))*0.06, ...
    'DROOP',  'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
ylim(ylims_rc_r)
legend('Location', 'northeast', 'FontSize', 9)

axes('position', [0.36, 0.57, 0.17, 0.35])
hold on, grid on
title({'RC & CP Lateral Position vs Heave'; ...
       ['RC mig: ', sprintf('%+.2f', rc_lat_migration_r), ' mm  |  ', ...
        'CP mig: ', sprintf('%+.2f', CP_y_migration_r), ' mm']}, ...
    'FontWeight', 'bold')
xlabel('Heave  [mm]  (+: jounce)')
ylabel('Lateral pos.  [mm]  (+ left of CL)')

cp_pad_r = max(max(abs(CP_r_y_sweep_r))*1000*0.01, 0.5);
y_lo_r   = min(CP_r_y_sweep_r)*1000 - cp_pad_r;
y_hi_r   = max(CP_r_y_sweep_r)*1000 + cp_pad_r;

fill([0, max(heave_rc_mm_r), max(heave_rc_mm_r), 0], ...
     [y_lo_r, y_lo_r, y_hi_r, y_hi_r], ...
     [1 0.9 0.9], 'EdgeColor','none','FaceAlpha',0.3,'HandleVisibility','off')
fill([min(heave_rc_mm_r), 0, 0, min(heave_rc_mm_r)], ...
     [y_lo_r, y_lo_r, y_hi_r, y_hi_r], ...
     [0.9 0.9 1], 'EdgeColor','none','FaceAlpha',0.3,'HandleVisibility','off')
plot(heave_rc_mm_r, CP_r_y_sweep_r*1000, 'b--', 'LineWidth', 2, 'DisplayName', 'CP Y - Right')
plot(heave_rc_mm_r, rc_y_r*1000,         'k-',  'LineWidth', 2, 'DisplayName', 'RC lateral pos.')
plot(heave_rc_mm_r(idx_des_rc_r), rc_y_design_r*1000, 'g*', 'MarkerSize', 12, 'LineWidth', 2, ...
    'DisplayName', sprintf('RC design: %+.2f mm', rc_y_design_r*1000))
plot(heave_rc_mm_r(idx_1g_r), rc_y_1g_r*1000, 'r^', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('RC at 1G: %+.2f mm', rc_y_1g_r*1000))
ylim([y_lo_r, y_hi_r])
yline(0, 'k--', 'LineWidth', 1, 'DisplayName', 'Centreline')
xline(0, 'k:',  'LineWidth', 1, 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 7.5)

axes('position', [0.57, 0.57, 0.17, 0.35])
hold on, grid on
title({'RC Height vs IC Lateral'; 'Migration path'}, 'FontWeight', 'bold')
xlabel('IC lateral  [mm]  (+ left)')
ylabel('RC height  [mm]')
scatter(IC_r_y_sweep_r*1000, rc_height_r*1000, 30, heave_rc_mm_r, 'filled', ...
    'DisplayName', 'Path (by heave)')
cbar_rc_r = colorbar;
cbar_rc_r.Label.String = 'Heave [mm]';
cbar_rc_r.Label.FontSize = 7;
cbar_rc_r.FontSize = 7;
cbar_rc_r.Position(3) = 0.008;
plot(IC_r_y_design_r*1000, rc_height_design_r*1000, 'g*', 'MarkerSize', 14, 'LineWidth', 2, ...
    'DisplayName', sprintf('Design (%.1f, %.1f)', IC_r_y_design_r*1000, rc_height_design_r*1000))
plot(IC_r_y_1g_r*1000, rc_height_1g_r*1000, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('1G (%.1f, %.1f)', IC_r_y_1g_r*1000, rc_height_1g_r*1000))
if abs(rc_vert_migration_r) > 0.01 || abs(IC_y_migration_r) > 0.01
    quiver(IC_r_y_design_r*1000, rc_height_design_r*1000, ...
           IC_y_migration_r, rc_vert_migration_r, 0, 'k-', 'LineWidth', 2, ...
           'MaxHeadSize', 0.5, 'DisplayName', ...
           sprintf('Mig (%+.1f, %+.1f) mm', IC_y_migration_r, rc_vert_migration_r))
end
yline(0, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off')
legend('Location', 'best', 'FontSize', 7)

axes('position', [0.78, 0.57, 0.20, 0.35])
axis off
title('RC Summary  |  Rear Axle', 'FontWeight', 'bold')

sum_rc_r = {
    'Parameter',                 'Value';
    'Design RC height',          sprintf('%.2f mm',  rc_height_design_r*1000);
    'RC height at 1G',           sprintf('%.2f mm',  rc_height_1g_r*1000);
    'RC vertical migration',     sprintf('%+.2f mm', rc_vert_migration_r);
    'RC lateral migration',      sprintf('%+.2f mm', rc_lat_migration_r);
    'Design IC lateral',         sprintf('%+.2f mm', IC_r_y_design_r*1000);
    'IC lateral at 1G',          sprintf('%+.2f mm', IC_r_y_1g_r*1000);
    'IC lateral migration',      sprintf('%+.2f mm', IC_y_migration_r);
    'CP lateral migration',      sprintf('%+.2f mm', CP_y_migration_r);
    'Total roll at 1G',          sprintf('%.3f deg', roll_angle_rc_deg_r);
    'Outer phi (jounce)',        sprintf('+%.3f deg', phi_rc_outer_r*180/pi);
};

col_x_rc_r = [0.01, 0.52];
row_y_rc_r  = 0.92;
row_dy_rc_r = 0.090;

for c = 1:2
    text(col_x_rc_r(c), row_y_rc_r, sum_rc_r{1,c}, ...
        'FontSize', 8, 'FontWeight', 'bold', 'Interpreter', 'none')
end
for r = 2:size(sum_rc_r,1)
    ry_r2 = row_y_rc_r - (r-1)*row_dy_rc_r;
    if mod(r,2) == 0
        patch([0, 1, 1, 0], [ry_r2-row_dy_rc_r*0.3, ry_r2-row_dy_rc_r*0.3, ...
               ry_r2+row_dy_rc_r*0.7, ry_r2+row_dy_rc_r*0.7], ...
              [0.93 0.93 0.93], 'EdgeColor', 'none', 'FaceAlpha', 0.6)
    end
    text(col_x_rc_r(1), ry_r2, sum_rc_r{r,1}, 'FontSize', 7.5, 'Interpreter', 'none')
    text(col_x_rc_r(2), ry_r2, sum_rc_r{r,2}, 'FontSize', 7.5, 'Interpreter', 'none', ...
        'FontWeight', 'bold')
end

axes('position', [0.05, 0.13, 0.65, 0.32])
hold on, grid on
title({'YZ Plane — Wishbone Geometry  |  Rear Axle  |  Design Position'; ...
       'Wishbone projections extended to instant centres  |  RC at vehicle centreline'}, ...
    'FontWeight', 'bold')
xlabel('y  [m]  (+: left,  -: right)')
ylabel('z  [m]  (+: upward)')

rvab_des_r   = (par_r.rvak + par_r.rvbk) / 2;
rvde_des_r   = (par_r.rvdk + par_r.rvek) / 2;
rvab_l_des_r = (par_rc_l_r.rvak + par_rc_l_r.rvbk) / 2;
rvde_l_des_r = (par_rc_l_r.rvdk + par_rc_l_r.rvek) / 2;

lower_dir_r   = [par_r.rvck(2)-rvab_des_r(2);       par_r.rvck(3)-rvab_des_r(3)];
upper_dir_r   = [par_r.rvfk(2)-rvde_des_r(2);       par_r.rvfk(3)-rvde_des_r(3)];
lower_dir_l_r = [par_rc_l_r.rvck(2)-rvab_l_des_r(2); par_rc_l_r.rvck(3)-rvab_l_des_r(3)];
upper_dir_l_r = [par_rc_l_r.rvfk(2)-rvde_l_des_r(2); par_rc_l_r.rvfk(3)-rvde_l_des_r(3)];
lower_dir_r   = lower_dir_r   / norm(lower_dir_r);
upper_dir_r   = upper_dir_r   / norm(upper_dir_r);
lower_dir_l_r = lower_dir_l_r / norm(lower_dir_l_r);
upper_dir_l_r = upper_dir_l_r / norm(upper_dir_l_r);

t_both_r = linspace(-2, 8, 2);

plot([rvab_des_r(2), par_r.rvck(2)], [rvab_des_r(3), par_r.rvck(3)], ...
    'b-', 'LineWidth', 2.5, 'DisplayName', 'Lower arm - Right')
plot([rvde_des_r(2), par_r.rvfk(2)], [rvde_des_r(3), par_r.rvfk(3)], ...
    'b--','LineWidth', 2.5, 'DisplayName', 'Upper arm - Right')
plot(par_r.rvck(2) + t_both_r*lower_dir_r(1), par_r.rvck(3) + t_both_r*lower_dir_r(2), ...
    'b:', 'LineWidth', 1, 'HandleVisibility', 'off')
plot(par_r.rvfk(2) + t_both_r*upper_dir_r(1), par_r.rvfk(3) + t_both_r*upper_dir_r(2), ...
    'b:', 'LineWidth', 1, 'HandleVisibility', 'off')
plot([rvab_l_des_r(2), par_rc_l_r.rvck(2)], [rvab_l_des_r(3), par_rc_l_r.rvck(3)], ...
    'r-', 'LineWidth', 2.5, 'DisplayName', 'Lower arm - Left')
plot([rvde_l_des_r(2), par_rc_l_r.rvfk(2)], [rvde_l_des_r(3), par_rc_l_r.rvfk(3)], ...
    'r--','LineWidth', 2.5, 'DisplayName', 'Upper arm - Left')
plot(par_rc_l_r.rvck(2) + t_both_r*lower_dir_l_r(1), ...
     par_rc_l_r.rvck(3) + t_both_r*lower_dir_l_r(2), ...
    'r:', 'LineWidth', 1, 'HandleVisibility', 'off')
plot(par_rc_l_r.rvfk(2) + t_both_r*upper_dir_l_r(1), ...
     par_rc_l_r.rvfk(3) + t_both_r*upper_dir_l_r(2), ...
    'r:', 'LineWidth', 1, 'HandleVisibility', 'off')

CP_rr_y = par_r.rvwk(2);      CP_rr_z = par_r.rvwk(3) - rs_r;
CP_lr_y = par_rc_l_r.rvwk(2); CP_lr_z = par_rc_l_r.rvwk(3) - rs_r;
plot(CP_rr_y, CP_rr_z, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'CP - Right')
plot(CP_lr_y, CP_lr_z, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'CP - Left')

plot(IC_r_static_r(1), IC_r_static_r(2), 'b+', 'MarkerSize', 14, 'LineWidth', 2.5, ...
    'DisplayName', sprintf('IC Right (%.0f, %.0f mm)', IC_r_static_r(1)*1000, IC_r_static_r(2)*1000))
plot(IC_l_static_r(1), IC_l_static_r(2), 'r+', 'MarkerSize', 14, 'LineWidth', 2.5, ...
    'DisplayName', sprintf('IC Left (%.0f, %.0f mm)', IC_l_static_r(1)*1000, IC_l_static_r(2)*1000))

t_rc_rr = -CP_rr_y / (IC_r_static_r(1) - CP_rr_y);
RC_z_rr = CP_rr_z + t_rc_rr*(IC_r_static_r(2) - CP_rr_z);
plot([CP_rr_y, IC_r_static_r(1)], [CP_rr_z, IC_r_static_r(2)], ...
    'b-.', 'LineWidth', 1.5, 'HandleVisibility', 'off')
plot([IC_r_static_r(1), 0], [IC_r_static_r(2), RC_z_rr], ...
    'b-.', 'LineWidth', 1.5, 'HandleVisibility', 'off')

plot(0, rc_height_design_r, 'g*', 'MarkerSize', 16, 'LineWidth', 2.5, ...
    'DisplayName', sprintf('Design RC: %.1f mm', rc_height_design_r*1000))
plot(0, rc_height_1g_r, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('1G RC: %.1f mm', rc_height_1g_r*1000))
plot(par_r.rvwk(2),      par_r.rvwk(3),      'bo', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.7 0.85 1], 'DisplayName', 'Wheel centre - Right')
plot(par_rc_l_r.rvwk(2), par_rc_l_r.rvwk(3), 'ro', 'MarkerSize', 8, ...
    'MarkerFaceColor', [1 0.8 0.8],  'DisplayName', 'Wheel centre - Left')

yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Ground')
xline(0, 'k-',  'LineWidth', 0.8, 'HandleVisibility', 'off')
xlim([-0.75, 0.75])
ylim([-0.05, 0.35])
legend('Location', 'eastoutside', 'FontSize', 8)

rc_ribbon_r = {
    ['Rear Axle  |  Roll gradient: ', num2str(roll_gradient_rc_r), ...
     ' deg/G  |  Lateral accel: ', num2str(lat_accel_rc_r), ' G  |  Total roll: ', ...
     sprintf('%.3f', roll_angle_rc_deg_r), ' deg'], ...
    ['Design RC: Z = ', sprintf('%.2f', rc_height_design_r*1000), ' mm  |  ', ...
     'RC at 1G: Z = ', sprintf('%.2f', rc_height_1g_r*1000), ' mm  |  ', ...
     'Vertical migration: ', sprintf('%+.2f', rc_vert_migration_r), ' mm  |  ', ...
     'RC lateral migration: ', sprintf('%+.2f', rc_lat_migration_r), ' mm'], ...
    'RC from wishbone YZ projections — body roll method (inboard points move with body)', ...
    'Outboard points follow kinematic solution  |  No compliance or tyre deflection'
};
add_sign_box_rear(rc_ribbon_r)

%% -----------------------------------------------------------------------
%  FIGURE 5: Toe-Camber Correlations + Summary Table
%% -----------------------------------------------------------------------
figure('Name', 'Figure 5 - Rear Toe-Camber Correlation & Summary')

axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Toe-Camber Correlation During Heave'; ['Rear-', wheel_side_r, '  |  Coloured by heave [mm]']}, 'FontWeight','bold')
xlabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
scatter(toe_right_chg, camb_right_chg, 80, heave_r, 'filled','o', 'DisplayName','Heave steps')
cbar = colorbar;
cbar.Label.String = 'Heave  h  [mm]  (+: jounce)';
cbar.Label.FontSize = 9;
plot(toe_right_chg(n0_r), camb_right_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({'Toe-Camber Correlation During Heave'; ['Rear-', rear_conditional_string(wheel_side_r), '  |  Coloured by heave [mm]']}, 'FontWeight','bold')
xlabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
scatter(toe_left_chg, camb_left_chg, 80, heave_r, 'filled','s', 'DisplayName','Heave steps')
cbar2 = colorbar;
cbar2.Label.String = 'Heave  h  [mm]  (+: jounce)';
cbar2.Label.FontSize = 9;
plot(toe_left_chg(n0_r), camb_left_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, 'DisplayName','Design pos.')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

axes('position',[0.15, 0.10, 0.70, 0.38])
axis off
title('Rear Axle Kinematics Summary  |  Heave Motion Only', 'FontWeight','bold')

n_rows_r  = 9;
key_idx_r = round(linspace(1, n_r, n_rows_r));

col_phi  = phi_r(key_idx_r)  * 180/pi;
col_hv   = heave_r(key_idx_r);
col_toeR = toe_right_chg(key_idx_r);
col_camR = camb_right_chg(key_idx_r);
col_toeL = toe_left_chg(key_idx_r);
col_camL = camb_left_chg(key_idx_r);

headers_r = {'\phi [deg]','Heave [mm]','\DeltaToe_R [deg]','\DeltaCamb_R [deg]', ...
             '\DeltaToe_L [deg]','\DeltaCamb_L [deg]'};
col_x_r  = [0.04, 0.19, 0.36, 0.52, 0.68, 0.84];
row_y_r  = 0.92;
row_dy_r = 0.095;

for c = 1:length(headers_r)
    text(col_x_r(c), row_y_r, headers_r{c}, ...
        'FontSize',8,'FontWeight','bold', ...
        'HorizontalAlignment','center','Interpreter','tex')
end
annotation('line',[0.15, 0.85],[0.435, 0.435],'Color','k','LineWidth',1.5)

for r = 1:length(key_idx_r)
    ry_r  = row_y_r - r * row_dy_r;
    vals_r = {sprintf('%.3f',  col_phi(r)), ...
              sprintf('%.2f',  col_hv(r)),  ...
              sprintf('%+.4f', col_toeR(r)), ...
              sprintf('%+.4f', col_camR(r)), ...
              sprintf('%+.4f', col_toeL(r)), ...
              sprintf('%+.4f', col_camL(r))};
    if mod(r,2) == 0
        annotation('rectangle',[0.15, ry_r-0.04, 0.70, row_dy_r], ...
            'FaceColor',[0.93 0.93 0.93],'EdgeColor','none','FaceAlpha',0.6)
    end
    for c = 1:length(vals_r)
        text(col_x_r(c), ry_r, vals_r{c}, ...
            'FontSize',8,'HorizontalAlignment','center','Interpreter','none')
    end
end

add_sign_box_rear(sign_block_r)

%% -----------------------------------------------------------------------
%  VALIDATION SUMMARY
%% -----------------------------------------------------------------------
fprintf('%s\n', repmat('=',1,70))
fprintf('REAR AXLE KINEMATICS - VALIDATION SUMMARY\n')
fprintf('%s\n', repmat('=',1,70))
fprintf('Jounce travel:         +%.4f deg\n', par_r.phmx_jounce*180/pi)
fprintf('Rebound travel:        -%.4f deg\n', par_r.phmx_droop*180/pi)
fprintf('Heave range:           %.2f to %.2f mm\n', min(heave_r), max(heave_r))
fprintf('\nRIGHT wheel:\n')
fprintf('  Toe range:           %+.4f to %+.4f deg\n', min(toe_right_chg), max(toe_right_chg))
fprintf('  Camber range:        %+.4f to %+.4f deg\n', min(camb_right_chg), max(camb_right_chg))
fprintf('  Toe @ design:        %+.4f deg\n', toe_design_R)
fprintf('  Camber @ design:     %+.4f deg\n', camb_design_R)
fprintf('\nLEFT wheel:\n')
fprintf('  Toe range:           %+.4f to %+.4f deg\n', min(toe_left_chg), max(toe_left_chg))
fprintf('  Camber range:        %+.4f to %+.4f deg\n', min(camb_left_chg), max(camb_left_chg))
fprintf('  Toe @ design:        %+.4f deg\n', toe_design_L)
fprintf('  Camber @ design:     %+.4f deg\n', camb_design_L)
fprintf('%s\n', repmat('=',1,70))

%% -----------------------------------------------------------------------
%  HELPERS
%% -----------------------------------------------------------------------
function opposite = rear_conditional_string(wheel_side)
    if strcmp(wheel_side, 'Right')
        opposite = 'Left';
    else
        opposite = 'Right';
    end
end

function add_sign_box_rear(sign_block_r)
    annotation('textbox', [0.00, 0.00, 1.00, 0.10], ...
        'String',           sign_block_r, ...
        'FontSize',         7.5, ...
        'FontName',         'Courier New', ...
        'BackgroundColor',  [0.95 0.95 0.85], ...
        'EdgeColor',        [0.40 0.40 0.20], ...
        'FitBoxToText',     'off', ...
        'LineWidth',        1, ...
        'Interpreter',      'none', ...
        'VerticalAlignment','middle');
end