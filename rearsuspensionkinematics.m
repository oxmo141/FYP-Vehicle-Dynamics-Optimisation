%% =========================================================================
%  Suspension Kinematics - Rear Axle (Heave Motion Only, No Steering)
%  =========================================================================
clear; clc
paramR26;

%% =========================================================================
%  >>>  USER PARAMETERS  <<<
%% =========================================================================

% -------------------------------------------------------------------------
%  1. SUSPENSION TRAVEL LIMITS
% -------------------------------------------------------------------------
par_r.phmx_droop  = 2.8306 / 180 * pi;
par_r.phmx_jounce = 6.007  / 180 * pi;

% -------------------------------------------------------------------------
%  2. REAR HARDPOINT COORDINATES  [x; y; z]  metres
%     ISO 8855: X forward, Y left, Z up
%     Negative Y = RIGHT wheel
% -------------------------------------------------------------------------
par_r.rvwk = [-0.66196;  -0.605;       0.2032   ];   % W  wheel rim centre
par_r.rvak = [-0.585;    -0.255;       0.115484 ];   % A  lower arm @ chassis rear
par_r.rvbk = [-0.470;    -0.284;       0.116    ];   % B  lower arm @ chassis front
par_r.rvck = [-0.63196;  -0.565;       0.121    ];   % C  lower arm @ wheel body
par_r.rvdk = [-0.5825;   -0.2465;      0.240343 ];   % D  upper arm @ chassis rear
par_r.rvek = [-0.470;    -0.290;       0.24749  ];   % E  upper arm @ chassis front
par_r.rvfk = [-0.63196;  -0.5111726;   0.283828 ];   % F  upper arm @ wheel body
par_r.rvgk = [-0.619;    -0.140;       0.1673839];   % G  toe link @ chassis
par_r.rvhk = [-0.75196;  -0.537826;    0.2032   ];   % H  toe link @ upright

% -------------------------------------------------------------------------
%  3. WHEEL / TYRE
% -------------------------------------------------------------------------
toe0_r  = 0 / 180 * pi;
camb0_r = 0 / 180 * pi;
rs_r    = car.r_wheel;

% -------------------------------------------------------------------------
%  4. DISCRETIZATION
% -------------------------------------------------------------------------
n_droop_r  = 10;
n_jounce_r = 15;

% -------------------------------------------------------------------------
%  5. WHEEL IDENTIFICATION
% -------------------------------------------------------------------------
wheel_side_r = 'Right';
conv_str_r   = 'ISO 8855';

%% =========================================================================
%  END OF USER PARAMETERS
%% =========================================================================

%% -----------------------------------------------------------------------
%  DERIVED QUANTITIES
%  -----------------------------------------------------------------------
en0 = [0; 0; 1];

eyrk_r = [toe0_r; 1; -camb0_r];  eyrk_r = eyrk_r / norm(eyrk_r);
exk_r  = cross(eyrk_r, en0);     exk_r  = exk_r  / norm(exk_r);
eyk_r  = cross(en0,    exk_r);
ezk_r  = cross(exk_r,  eyrk_r);
rwpk_r = -rs_r * ezk_r;

% =========================================================================
%  Kingpin inclination and caster
% =========================================================================
rcfk_r = par_r.rvfk - par_r.rvck;
ecfk_r = rcfk_r / norm(rcfk_r);

si_r = atan2(ecfk_r(2), ecfk_r(3));
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

%% -----------------------------------------------------------------------
%  SIGN CONVENTION ANNOTATION BLOCK
%  -----------------------------------------------------------------------
sign_block_r = {
    ['Analyzed Wheel: Rear-', wheel_side_r, ' (negative Y)'], ...
    ['Opposite Wheel: Rear-', rear_conditional_string(wheel_side_r)], ...
    ['Coord. system: ', conv_str_r, ' (X fwd, Y left, Z up)'], ...
    'Toe  (+): toe-in  (+Z rotation)', ...
    'Camber (+): top outboard  (+X rotation)', ...
    'Phi  (+): jounce/compression  |  Phi (-): droop/rebound', ...
    'No steering input - heave motion only'
};

%% -----------------------------------------------------------------------
%  DISCRETIZATION
%  -----------------------------------------------------------------------
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

%% -----------------------------------------------------------------------
%  PREPARE LEFT WHEEL HARDPOINTS (MIRROR par_r)
%  Do this BEFORE any kinematics calculations
%% -----------------------------------------------------------------------
par_l = par_r;  % Copy right wheel hardpoints
fields_mirror = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk','rvgk','rvhk'};
for f = 1:length(fields_mirror)
    par_l.(fields_mirror{f})(2) = -par_r.(fields_mirror{f})(2);  % Mirror Y-coordinate
end

% =========================================================================
%  RIGHT WHEEL — sweep phi, carry toe_prev forward
% =========================================================================
toe_right  = zeros(n_r, 1);
camb_right = zeros(n_r, 1);
xw_r  = zeros(n_r, 1);
yw_r  = zeros(n_r, 1);
zw_r  = zeros(n_r, 1);
xp_r  = zeros(n_r, 1);
yp_r  = zeros(n_r, 1);
zp_r  = zeros(n_r, 1);

toe_prev_r = 0;  % seed at design position

for i = 1:n_r
    [avw_r, rvwv_r, toe_right(i), camb_right(i)] = ...
        fun_rear_kin(phi_r(i), par_r, toe_prev_r);

    toe_prev_r = toe_right(i);  % carry forward for next step

    rvpv_r = rvwv_r + avw_r * rwpk_r;
    xw_r(i) = rvwv_r(1);
    yw_r(i) = rvwv_r(2);
    zw_r(i) = rvwv_r(3);
    xp_r(i) = rvpv_r(1);
    yp_r(i) = rvpv_r(2);
    zp_r(i) = rvpv_r(3);
end

% =========================================================================
%  LEFT WHEEL — INDEPENDENT toe_prev, reset to 0
%  Uses mirrored par_l (already prepared above)
% =========================================================================
toe_left  = zeros(n_r, 1);
camb_left = zeros(n_r, 1);

toe_prev_l = 0;  % seed INDEPENDENTLY from right wheel

for i = 1:n_r
    [~, ~, toe_left(i), camb_left(i)] = ...
        fun_rear_kin(phi_r(i), par_l, toe_prev_l);

    toe_prev_l = toe_left(i);  % carry forward for next step
end


%% ===== HARDPOINT SYMMETRY CHECK =====
fprintf('\n=== HARDPOINT POSITIONS (WORLD FRAME) ===\n\n')
fprintf('%-8s | %12s | %12s | %12s | %12s\n', 'Point', 'Right X', 'Left X', 'Right Y', 'Left Y')
fprintf('%s\n', repmat('-',80,1))

points_r = struct('A', par_r.rvak, 'B', par_r.rvbk, 'C', par_r.rvck, ...
                  'D', par_r.rvdk, 'E', par_r.rvek, 'F', par_r.rvfk, ...
                  'G', par_r.rvgk, 'H', par_r.rvhk, 'W', par_r.rvwk);
points_l = struct('A', par_l.rvak, 'B', par_l.rvbk, 'C', par_l.rvck, ...
                  'D', par_l.rvdk, 'E', par_l.rvek, 'F', par_l.rvfk, ...
                  'G', par_l.rvgk, 'H', par_l.rvhk, 'W', par_l.rvwk);

names = fieldnames(points_r);
for i = 1:length(names)
    pt_name = names{i};
    pt_r = points_r.(pt_name);
    pt_l = points_l.(pt_name);
    
    fprintf('%-8s | %+.6f | %+.6f | %+.6f | %+.6f\n', ...
            pt_name, pt_r(1), pt_l(1), pt_r(2), pt_l(2))
end

%% Check if mirroring is correct
fprintf('\n=== MIRRORING VERIFICATION ===\n')
fprintf('Point | X match? | Y opposite? | Z match?\n')
fprintf('%s\n', repmat('-',50,1))
for i = 1:length(names)
    pt_name = names{i};
    pt_r = points_r.(pt_name);
    pt_l = points_l.(pt_name);
    
    x_match = abs(pt_r(1) - pt_l(1)) < 1e-6;
    y_opposite = abs(pt_r(2) + pt_l(2)) < 1e-6;  % Y should sum to zero
    z_match = abs(pt_r(3) - pt_l(3)) < 1e-6;
    
    fprintf('%s | %s | %s | %s\n', ...
            pt_name, ...
            char(pass_fail(x_match)), ...
            char(pass_fail(y_opposite)), ...
            char(pass_fail(z_match)))
end

function result = pass_fail(condition)
    if condition
        result = '✓';
    else
        result = '✗ FAIL';
    end
end
%% -----------------------------------------------------------------------
%  NEUTRAL POSITION DIAGNOSTIC (exact phi=0)
%  -----------------------------------------------------------------------
[~, ~, toe_neut_R_rad, camb_neut_R_rad] = fun_rear_kin(0, par_r);
[~, ~, toe_neut_L_rad, camb_neut_L_rad] = fun_rear_kin(0, par_l);

toe_R_neut  = toe_neut_R_rad  * 180/pi;
toe_L_neut  = toe_neut_L_rad  * 180/pi;
camb_R_neut = camb_neut_R_rad * 180/pi;
camb_L_neut = camb_neut_L_rad * 180/pi;

fprintf('\n%s\n', repmat('=',1,70))
fprintf('REAR AXLE - NEUTRAL POSITION ANALYSIS (exact phi=0)\n')
fprintf('%s\n', repmat('=',1,70))
fprintf('Toe   Angle - Right:  %+.4f deg\n', toe_R_neut)
fprintf('Toe   Angle - Left:   %+.4f deg\n', toe_L_neut)
fprintf('Toe   Angle - Mean:   %+.4f deg\n', mean([toe_R_neut, toe_L_neut]))
fprintf('\nCamber Angle - Right: %+.4f deg\n', camb_R_neut)
fprintf('Camber Angle - Left:  %+.4f deg\n', camb_L_neut)
fprintf('Camber Angle - Mean:  %+.4f deg\n', mean([camb_R_neut, camb_L_neut]))

toe_asym  = abs(toe_R_neut  - toe_L_neut);
camb_asym = abs(camb_R_neut - camb_L_neut);

if toe_asym > 0.1
    fprintf('\nWARNING: Toe asymmetry at neutral: %.4f deg\n', toe_asym)
else
    fprintf('\nOK: Toe symmetry at neutral: %.4f deg\n', toe_asym)
end
if camb_asym > 0.3
    fprintf('WARNING: Camber asymmetry at neutral: %.4f deg\n', camb_asym)
else
    fprintf('OK: Camber symmetry at neutral: %.4f deg\n', camb_asym)
end
fprintf('%s\n\n', repmat('=',1,70))

%% -----------------------------------------------------------------------
%  HEAVE EXCURSION
%  -----------------------------------------------------------------------
z_design_r   = zw_r(n0_r);
heave_r      = (zw_r - z_design_r) * 1000;   % [mm]

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
%  -----------------------------------------------------------------------
figure('Name', ['Figure 1 - Rear Wheel Centre Paths  |  Analyzed: ', wheel_side_r])

% XZ Plane — remove axis equal, use padded auto limits
axes('position',[0.05, 0.12, 0.18, 0.78])
hold on, grid on
title({'Longitudinal Plane (XZ)'; ['Rear-', wheel_side_r]}, 'FontWeight','bold')
xlabel('x  [m]  (+ forward)')
ylabel('z  [m]  (+ upward)')
plot(xw_r, zw_r, 'b-',  'LineWidth',1.5, 'DisplayName','W centre')
plot(xp_r, zp_r, 'b--', 'LineWidth',1.5, 'DisplayName','Contact P')
plot(par_r.rvwk(1), par_r.rvwk(3), 'ok', 'MarkerFaceColor','k',   'MarkerSize',6, 'DisplayName','W design')
plot(rvpk_r(1),     rvpk_r(3),     'o',  'Color','cyan', ...
     'MarkerFaceColor','cyan','MarkerSize',6, 'DisplayName','P design')
% *** KEY FIX: pad axes proportionally, NOT axis equal ***
x_pad = max(0.002, 0.5*(max(xw_r)-min(xw_r)));
z_pad = max(0.005, 0.1*(max(zw_r)-min(zw_r)));
xlim([min(xw_r)-x_pad, max(xw_r)+x_pad])
ylim([min(zw_r)-z_pad, max(zw_r)+z_pad])
legend('Location','best','FontSize',8)

% YZ Plane — same fix
axes('position',[0.28, 0.12, 0.18, 0.78])
hold on, grid on
title({'Lateral Plane (YZ)'; ['Rear-', wheel_side_r]}, 'FontWeight','bold')
xlabel({'y  [m]'; '(+left, -Y=right)'})
ylabel('z  [m]  (+ upward)')
plot(yw_r, zw_r, 'r-',  'LineWidth',1.5, 'DisplayName','W centre')
plot(yp_r, zp_r, 'r--', 'LineWidth',1.5, 'DisplayName','Contact P')
plot(par_r.rvwk(2), par_r.rvwk(3), 'ok', 'MarkerFaceColor','k',   'MarkerSize',6, 'DisplayName','W design')
plot(rvpk_r(2),     rvpk_r(3),     'o',  'Color','cyan', ...
     'MarkerFaceColor','cyan','MarkerSize',6, 'DisplayName','P design')
y_pad = max(0.002, 0.5*(max(yw_r)-min(yw_r)));
z_pad = max(0.005, 0.1*(max(zw_r)-min(zw_r)));
xlim([min(yw_r)-y_pad, max(yw_r)+y_pad])
ylim([min(zw_r)-z_pad, max(zw_r)+z_pad])
legend('Location','best','FontSize',8)

% Toe vs Phi
axes('position',[0.55, 0.55, 0.42, 0.35])
hold on, grid on
title({'Toe Angle vs Lower Arm Rotation'; '(+) = toe-in  |  No steering'}, ...
    'FontWeight','bold')
xlabel('\phi  [deg]  (+: jounce,  -: droop)')
ylabel('Toe  [deg]  (+: toe-in)')
plot(phi_r*180/pi, toe_right*180/pi, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Right (', wheel_side_r, ')'])
plot(phi_r*180/pi, toe_left*180/pi,  'r--s','LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Left (', rear_conditional_string(wheel_side_r), ')'])
plot(phi_r(n0_r)*180/pi, toe_right(n0_r)*180/pi, 'g*', 'MarkerSize',14, ...
    'LineWidth',2,'HandleVisibility','off')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
legend('Location','best','FontSize',8)

% Camber vs Phi
axes('position',[0.55, 0.12, 0.42, 0.35])
hold on, grid on
title({'Camber Angle vs Lower Arm Rotation'; '(+) = top outboard  |  No steering'}, ...
    'FontWeight','bold')
xlabel('\phi  [deg]  (+: jounce,  -: droop)')
ylabel('Camber  [deg]  (+: top outboard)')
plot(phi_r*180/pi, camb_right*180/pi, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Right (', wheel_side_r, ')'])
plot(phi_r*180/pi, camb_left*180/pi,  'r--s','LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Left (', rear_conditional_string(wheel_side_r), ')'])
plot(phi_r(n0_r)*180/pi, camb_right(n0_r)*180/pi, 'g*', 'MarkerSize',14, ...
    'LineWidth',2,'HandleVisibility','off')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
legend('Location','best','FontSize',8)

add_sign_box_rear(sign_block_r)

%% -----------------------------------------------------------------------
%  FIGURE 2: Toe & Camber During Heave - Both Wheels
%  -----------------------------------------------------------------------
figure('Name', 'Figure 2 - Rear Heave Response  |  Both Wheels')

% Right Toe vs Heave
axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({['Toe Change vs Heave  |  Rear-', wheel_side_r]; 'No steering  (u = 0)'}, ...
    'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')

tj_R = toe_right_chg(jounce_idx_r);
td_R = toe_right_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [tj_R; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [td_R; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, toe_right_chg, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Toe change - Right (', wheel_side_r, ')'])
plot(heave_r(n0_r), toe_right_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
if max(abs(toe_right_chg)) > 0
    text(max(heave_r)*0.1, max(abs(toe_right_chg))*0.75, 'JOUNCE', ...
        'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
    text(min(heave_r)*0.9, max(abs(toe_right_chg))*0.75, 'DROOP', ...
        'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
end
legend('Location','southwest','FontSize',9)

% Right Camber vs Heave
axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Rear-', wheel_side_r]; 'No steering  (u = 0)'}, ...
    'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')

cj_R = camb_right_chg(jounce_idx_r);
cd_R = camb_right_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [cj_R; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [cd_R; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, camb_right_chg, 'r-s', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Camber change - Right (', wheel_side_r, ')'])
plot(heave_r(n0_r), camb_right_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
if max(abs(camb_right_chg)) > 0
    text(max(heave_r)*0.1, max(abs(camb_right_chg))*0.75, 'JOUNCE', ...
        'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
    text(min(heave_r)*0.9, max(abs(camb_right_chg))*0.75, 'DROOP', ...
        'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
end
legend('Location','southwest','FontSize',9)

% Left Toe vs Heave
axes('position',[0.06, 0.14, 0.38, 0.33])
hold on, grid on
title({['Toe Change vs Heave  |  Rear-', rear_conditional_string(wheel_side_r)]; ...
       'No steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')

tj_L = toe_left_chg(jounce_idx_r);
td_L = toe_left_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [tj_L; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [td_L; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, toe_left_chg, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Toe change - Left (', rear_conditional_string(wheel_side_r), ')'])
plot(heave_r(n0_r), toe_left_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
if max(abs(toe_left_chg)) > 0
    text(max(heave_r)*0.1, max(abs(toe_left_chg))*0.75, 'JOUNCE', ...
        'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
    text(min(heave_r)*0.9, max(abs(toe_left_chg))*0.75, 'DROOP', ...
        'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
end
legend('Location','southwest','FontSize',9)

% Left Camber vs Heave
axes('position',[0.56, 0.14, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Rear-', rear_conditional_string(wheel_side_r)]; ...
       'No steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')

cj_L = camb_left_chg(jounce_idx_r);
cd_L = camb_left_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [cj_L; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [cd_L; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, camb_left_chg, 'r-s', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Camber change - Left (', rear_conditional_string(wheel_side_r), ')'])
plot(heave_r(n0_r), camb_left_chg(n0_r), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
if max(abs(camb_left_chg)) > 0
    text(max(heave_r)*0.1, max(abs(camb_left_chg))*0.75, 'JOUNCE', ...
        'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
    text(min(heave_r)*0.9, max(abs(camb_left_chg))*0.75, 'DROOP', ...
        'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
end
legend('Location','southwest','FontSize',9)

add_sign_box_rear(sign_block_r)

%% -----------------------------------------------------------------------
%  FIGURE 3: Toe-Camber Correlations + Summary Table
%  -----------------------------------------------------------------------
figure('Name', 'Figure 3 - Rear Toe-Camber Correlation & Summary')

% Right correlation
axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Toe-Camber Correlation During Heave'; ...
       ['Rear-', wheel_side_r, '  |  Coloured by heave [mm]']}, 'FontWeight','bold')
xlabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
scatter(toe_right_chg, camb_right_chg, 80, heave_r, 'filled','o', ...
    'DisplayName','Heave steps')
cbar = colorbar;
cbar.Label.String  = 'Heave  h  [mm]  (+: jounce)';
cbar.Label.FontSize = 9;
plot(toe_right_chg(n0_r), camb_right_chg(n0_r), 'g*', 'MarkerSize',14, ...
    'LineWidth',2,'DisplayName','Design pos.')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

% Left correlation
axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({'Toe-Camber Correlation During Heave'; ...
       ['Rear-', rear_conditional_string(wheel_side_r), '  |  Coloured by heave [mm]']}, ...
    'FontWeight','bold')
xlabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
scatter(toe_left_chg, camb_left_chg, 80, heave_r, 'filled','s', ...
    'DisplayName','Heave steps')
cbar2 = colorbar;
cbar2.Label.String  = 'Heave  h  [mm]  (+: jounce)';
cbar2.Label.FontSize = 9;
plot(toe_left_chg(n0_r), camb_left_chg(n0_r), 'g*', 'MarkerSize',14, ...
    'LineWidth',2,'DisplayName','Design pos.')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

% Summary Table
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
%  -----------------------------------------------------------------------
fprintf('%s\n', repmat('=',1,70))
fprintf('REAR AXLE KINEMATICS - VALIDATION SUMMARY\n')
fprintf('%s\n', repmat('=',1,70))
fprintf('Jounce travel:         +%.4f deg\n', par_r.phmx_jounce*180/pi)
fprintf('Rebound travel:        -%.4f deg\n', par_r.phmx_droop*180/pi)
fprintf('Total phi steps:        %d\n',       n_r)
fprintf('Heave range:           %.2f to %.2f mm\n', min(heave_r), max(heave_r))
fprintf('\n')
fprintf('RIGHT wheel:\n')
fprintf('  Toe range:           %+.4f to %+.4f deg\n', ...
        min(toe_right_chg), max(toe_right_chg))
fprintf('  Camber range:        %+.4f to %+.4f deg\n', ...
        min(camb_right_chg), max(camb_right_chg))
fprintf('  Toe @ design pos:    %+.4f deg (absolute)\n', toe_design_R)
fprintf('  Camber @ design pos: %+.4f deg (absolute)\n', camb_design_R)
fprintf('\nLEFT wheel:\n')
fprintf('  Toe range:           %+.4f to %+.4f deg\n', ...
        min(toe_left_chg), max(toe_left_chg))
fprintf('  Camber range:        %+.4f to %+.4f deg\n', ...
        min(camb_left_chg), max(camb_left_chg))
fprintf('  Toe @ design pos:    %+.4f deg (absolute)\n', toe_design_L)
fprintf('  Camber @ design pos: %+.4f deg (absolute)\n', camb_design_L)
fprintf('%s\n', repmat('=',1,70))

% Wheel frame axes at design
disp(' ')
disp(['Wheel X-axis (longitudinal): ', num2str(exk_r')])
disp(['Wheel Y-axis (lateral):      ', num2str(eyk_r')])
disp(['Wheel Z-axis (vertical):     ', num2str(ezk_r')])

toe_link_vec = par_r.rvhk - par_r.rvgk;
disp(['Toe link vector (G->H):      ', num2str(toe_link_vec')])
toe_x = exk_r' * toe_link_vec;
toe_y = eyk_r' * toe_link_vec;
toe_z = ezk_r' * toe_link_vec;
disp(['Toe link in wheel frame:     X=', num2str(toe_x,'%.5f'), ...
      '  Y=', num2str(toe_y,'%.5f'), '  Z=', num2str(toe_z,'%.5f')])

%% -----------------------------------------------------------------------
%  HELPER: Opposite wheel side string
%% -----------------------------------------------------------------------
function opposite = rear_conditional_string(wheel_side)
    if strcmp(wheel_side, 'Right')
        opposite = 'Left';
    else
        opposite = 'Right';
    end
end

%% -----------------------------------------------------------------------
%  HELPER: Sign convention ribbon
%% -----------------------------------------------------------------------
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
