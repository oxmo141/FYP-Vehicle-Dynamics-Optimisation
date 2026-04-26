%% =========================================================================
%  Suspension Kinematics — Rear Axle (Heave Motion Only, No Steering)
%  =========================================================================
clear; clc
paramR26;

%% =========================================================================
%  >>>  USER PARAMETERS — EDIT HERE  <<<
%% =========================================================================

% -------------------------------------------------------------------------
%  1. SUSPENSION TRAVEL LIMITS (rear axle)
% -------------------------------------------------------------------------
par_r.phmx_droop  = 2.8306 / 180 * pi;   % [rad] max lower arm rotation — droop/rebound
par_r.phmx_jounce = 6.007  / 180 * pi;   % [rad] max lower arm rotation — jounce/compression

% -------------------------------------------------------------------------
%  2. REAR HARDPOINT COORDINATES  [x; y; z]  in metres
%     ISO 8855: X forward, Y left, Z up
%     Negative Y => RIGHT-hand side wheel (analyzed wheel)
%     *** REPLACE WITH YOUR ACTUAL REAR HARDPOINTS ***
% -------------------------------------------------------------------------
par_r.rvwk = [-0.66196;  -0.605;      0.2032   ];  % W  wheel rim centre
par_r.rvak = [-0.585;  -0.255;      0.115484 ];  % A  lower arm @ chassis rear
par_r.rvbk = [-0.470;  -0.284;      0.116 ];  % B  lower arm @ chassis front
par_r.rvck = [-0.63196;  -0.565;     0.121 ];  % C  lower arm @ wheel body
par_r.rvdk = [-0.5825;  -0.2465;     0.240343];  % D  upper arm @ chassis rear
par_r.rvek = [-0.470;  -0.290;     0.24749];  % E  upper arm @ chassis front
par_r.rvfk = [-0.63196;  -0.5111726;  0.283828   ];  % F  upper arm @ wheel body
%  NOTE: No drag link / rack hardpoints (R, Q) — rear has no steering input

% -------------------------------------------------------------------------
%  3. WHEEL / TYRE INITIAL CONDITIONS
% -------------------------------------------------------------------------
toe0_r  = 0 / 180 * pi;   % [rad]  initial toe angle   (ISO 8855, + = toe-in)
camb0_r = 0 / 180 * pi;   % [rad]  initial camber angle (ISO 8855, + = top outboard)
rs_r    = car.r_wheel;    % [m]    steady-state tyre radius

% -------------------------------------------------------------------------
%  4. DISCRETIZATION
% -------------------------------------------------------------------------
n_droop_r  = 10;   % droop steps
n_jounce_r = 15;   % jounce steps

% -------------------------------------------------------------------------
%  5. WHEEL IDENTIFICATION
% -------------------------------------------------------------------------
wheel_side_r = 'Right';    % Analyzed wheel (negative Y hardpoints)
conv_str_r   = 'ISO 8855';

%% =========================================================================
%  END OF USER PARAMETERS
%% =========================================================================

%% -----------------------------------------------------------------------
%  DERIVED QUANTITIES
%  -----------------------------------------------------------------------
en0 = [0; 0; 1];   % road normal

% Wheel orientation unit vectors at design position
eyrk_r = [toe0_r; 1; -camb0_r];  eyrk_r = eyrk_r / norm(eyrk_r);
exk_r  = cross(eyrk_r, en0);     exk_r  = exk_r  / norm(exk_r);
eyk_r  = cross(en0,    exk_r);
ezk_r  = cross(exk_r,  eyrk_r);
rwpk_r = -rs_r * ezk_r;           % W --> P (contact point offset)

% Kingpin inclination (sigma) and caster angle (nu)
rcfk_r = par_r.rvfk - par_r.rvck;
ecfk_r = rcfk_r / norm(rcfk_r);
si_r   = atan2(-ecfk_r(2), ecfk_r(3));
nu_r   = atan2(-ecfk_r(1), ecfk_r(3));
disp(['[REAR] sigma (KPI)  = ', num2str(si_r * 180/pi, '%.3f'), ' deg'])
disp(['[REAR] nu   (caster)= ', num2str(nu_r * 180/pi, '%.3f'), ' deg'])

% Caster offset and scrub radius
rcpk_r  = par_r.rvwk + rwpk_r - par_r.rvck;
rsck_r  = -(en0.' * rcpk_r) / (en0.' * ecfk_r) * ecfk_r;
co_r    = -exk_r' * (rsck_r + rcpk_r);
sr_r    =  eyk_r' * (rsck_r + rcpk_r);
disp(['[REAR] caster offset = ', num2str(co_r,  '%.4f')])
disp(['[REAR] scrub radius  = ', num2str(sr_r,  '%.4f')])

rvpk_r = par_r.rvwk + rwpk_r;   % Contact point at design position

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
    'No steering input — heave motion only'
};

%% -----------------------------------------------------------------------
%  SUSPENSION TRAVEL DISCRETIZATION (asymmetric)
%  -----------------------------------------------------------------------
n_r = n_droop_r + n_jounce_r - 1;

phi_droop_r  = linspace(-par_r.phmx_droop,  0, n_droop_r);
phi_jounce_r = linspace(0, par_r.phmx_jounce, n_jounce_r);
phi_r        = [phi_droop_r(1:end-1), phi_jounce_r];

[~, n0_r] = min(abs(phi_r));   % design position index

disp(' ')
disp('Rear suspension travel discretization:')
disp(['  Droop:  ', num2str(n_droop_r),  ' steps,  ', ...
      num2str(-par_r.phmx_droop  * 180/pi, '%.4f'), ' deg'])
disp(['  Jounce: ', num2str(n_jounce_r), ' steps, +', ...
      num2str( par_r.phmx_jounce * 180/pi, '%.4f'), ' deg'])
disp(['  Total:  ', num2str(n_r), ' steps'])
disp(['  Design position at index n0_r = ', num2str(n0_r), ...
      '  (phi = ', num2str(phi_r(n0_r)*180/pi, '%.3f'), ' deg)'])

%% -----------------------------------------------------------------------
%  COMPUTE KINEMATICS — RIGHT WHEEL (analyzed)
%  No steering: u = 0 throughout
%  -----------------------------------------------------------------------
u_rear = 0;   % Fixed: no rack displacement

xw_r = zeros(n_r,1);  yw_r = xw_r;  zw_r = xw_r;
xp_r = xw_r;          yp_r = xw_r;  zp_r = xw_r;
toe_right  = zeros(n_r,1);
camb_right = zeros(n_r,1);

for i = 1:n_r
    [avw_r, rvwv_r, toe_right(i), camb_right(i)] = fun_rear_kin(phi_r(i), par_r);
    rvpv_r    = rvwv_r + avw_r * rwpk_r;
    xw_r(i)   = rvwv_r(1);  yw_r(i) = rvwv_r(2);  zw_r(i) = rvwv_r(3);
    xp_r(i)   = rvpv_r(1);  yp_r(i) = rvpv_r(2);  zp_r(i) = rvpv_r(3);
end

%% -----------------------------------------------------------------------
%  COMPUTE KINEMATICS — LEFT WHEEL (mirror of hardpoints)
%  -----------------------------------------------------------------------
par_r_opp = par_r;
fields_r  = {'rvwk','rvak','rvbk','rvck','rvdk','rvek','rvfk'};
for f = 1:length(fields_r)
    par_r_opp.(fields_r{f})(2) = -par_r.(fields_r{f})(2);   % Mirror Y
end

toe_left  = zeros(n_r,1);
camb_left = zeros(n_r,1);

for i = 1:n_r
    [~, ~, toe_left(i), camb_left(i)] = fun_rear_kin(phi_r(i), par_r_opp);
end

%% -----------------------------------------------------------------------
%  HEAVE EXCURSION (relative to design position)
%  -----------------------------------------------------------------------
z_design_r   = zw_r(n0_r);
heave_r      = (zw_r - z_design_r) * 1000;   % [mm]

% Changes relative to design position
toe_right_chg  = (toe_right  - toe_right(n0_r))  * 180/pi;  % [deg]
camb_right_chg = (camb_right - camb_right(n0_r)) * 180/pi;  % [deg]
toe_left_chg   = (toe_left   - toe_left(n0_r))   * 180/pi;  % [deg]
camb_left_chg  = (camb_left  - camb_left(n0_r))  * 180/pi;  % [deg]

% Absolute angles at design position
toe_design_R   = toe_right(n0_r)  * 180/pi;
camb_design_R  = camb_right(n0_r) * 180/pi;
toe_design_L   = toe_left(n0_r)   * 180/pi;
camb_design_L  = camb_left(n0_r)  * 180/pi;

% Jounce / droop index split
jounce_idx_r = find(heave_r >= 0);
droop_idx_r  = find(heave_r <= 0);

hj_r = heave_r(jounce_idx_r);
hd_r = heave_r(droop_idx_r);

%% -----------------------------------------------------------------------
%  NEUTRAL POSITION DIAGNOSTIC (exact phi=0)
%  -----------------------------------------------------------------------
[~, ~, toe_neut_R_rad, camb_neut_R_rad] = fun_rear_kin(0, par_r);
camb_R_neut = camb_neut_R_rad * 180/pi;
toe_R_neut  = toe_neut_R_rad  * 180/pi;

[~, ~, toe_neut_L_rad, camb_neut_L_rad] = fun_rear_kin(0, par_r_opp);
camb_L_neut = camb_neut_L_rad * 180/pi;
toe_L_neut  = toe_neut_L_rad  * 180/pi;

fprintf('\n%s\n', repmat('=',1,70))
fprintf('REAR AXLE — NEUTRAL POSITION ANALYSIS (exact phi=0)\n')
fprintf('%s\n', repmat('=',1,70))
fprintf('Toe   Angle - Right:  %+.4f deg\n', toe_R_neut)
fprintf('Toe   Angle - Left:   %+.4f deg\n', toe_L_neut)
fprintf('Toe   Angle - Mean:   %+.4f deg\n', mean([toe_R_neut, toe_L_neut]))
fprintf('\nCamber Angle - Right: %+.4f deg\n', camb_R_neut)
fprintf('Camber Angle - Left:  %+.4f deg\n', camb_L_neut)
fprintf('Camber Angle - Mean:  %+.4f deg\n', mean([camb_R_neut, camb_L_neut]))

if abs(toe_R_neut - toe_L_neut) > 0.1
    fprintf('\nWARNING: Toe asymmetry at neutral: %.4f deg\n', ...
            abs(toe_R_neut - toe_L_neut))
end
if abs(camb_R_neut - camb_L_neut) > 0.3
    fprintf('\nWARNING: Camber asymmetry at neutral: %.4f deg\n', ...
            abs(camb_R_neut - camb_L_neut))
end
fprintf('%s\n\n', repmat('=',1,70))

%% -----------------------------------------------------------------------
%  FIGURE 1: Wheel Centre Paths + Toe & Camber Line Plots
%  (Rear equivalent of front Fig 1 — no surface maps since u=0 fixed)
%  -----------------------------------------------------------------------
figure('Name', ['Figure 1 - Rear Wheel Centre Paths  |  Analyzed: ', wheel_side_r, ' Wheel'])

% XZ Plane
axes('position',[0.05, 0.12, 0.18, 0.78])
hold on, axis equal, grid on
title({'Longitudinal Plane (XZ)'; ['Rear-', wheel_side_r]}, 'FontWeight','bold')
xlabel('x  [m]  (+ forward)')
ylabel('z  [m]  (+ upward)')
h1 = plot(xw_r, zw_r, 'b-',  'LineWidth',1.5);
h2 = plot(xp_r, zp_r, 'b--', 'LineWidth',1.5);
h3 = plot(par_r.rvwk(1), par_r.rvwk(3), 'ok', 'MarkerFaceColor','k',   'MarkerSize',6);
h4 = plot(rvpk_r(1),     rvpk_r(3),     'o',  'Color','cyan', ...
          'MarkerFaceColor','cyan','MarkerSize',6);
legend([h1,h2,h3,h4], 'W centre','Contact P','W design','P design', ...
    'Location','best','FontSize',8)

% YZ Plane
axes('position',[0.28, 0.12, 0.18, 0.78])
hold on, axis equal, grid on
title({'Lateral Plane (YZ)'; ['Rear-', wheel_side_r]}, 'FontWeight','bold')
xlabel({'y  [m]'; '(+left, -Y=right)'})
ylabel('z  [m]  (+ upward)')
h1 = plot(yw_r, zw_r, 'r-',  'LineWidth',1.5);
h2 = plot(yp_r, zp_r, 'r--', 'LineWidth',1.5);
h3 = plot(par_r.rvwk(2), par_r.rvwk(3), 'ok', 'MarkerFaceColor','k',   'MarkerSize',6);
h4 = plot(rvpk_r(2),     rvpk_r(3),     'o',  'Color','cyan', ...
          'MarkerFaceColor','cyan','MarkerSize',6);
legend([h1,h2,h3,h4], 'W centre','Contact P','W design','P design', ...
    'Location','best','FontSize',8)

% Toe vs Phi (top-right)
axes('position',[0.55, 0.55, 0.42, 0.35])
hold on, grid on
title({'Toe Angle vs Lower Arm Rotation'; '(+) = toe-in  |  No steering input'}, ...
    'FontWeight','bold')
xlabel('\phi  [deg]  (+: jounce,  -: droop)')
ylabel('Toe  [deg]  (+: toe-in)')
plot(phi_r*180/pi, toe_right*180/pi, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Right (',wheel_side_r,')'])
plot(phi_r*180/pi, toe_left*180/pi,  'r--s','LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Left (',rear_conditional_string(wheel_side_r),')'])
plot(phi_r(n0_r)*180/pi, toe_right(n0_r)*180/pi, 'g*', 'MarkerSize',14, ...
    'LineWidth',2,'HandleVisibility','off')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
legend('Location','best','FontSize',8)

% Camber vs Phi (bottom-right)
axes('position',[0.55, 0.12, 0.42, 0.35])
hold on, grid on
title({'Camber Angle vs Lower Arm Rotation'; '(+) = top outboard  |  No steering input'}, ...
    'FontWeight','bold')
xlabel('\phi  [deg]  (+: jounce,  -: droop)')
ylabel('Camber  [deg]  (+: top outboard)')
plot(phi_r*180/pi, camb_right*180/pi, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Right (',wheel_side_r,')'])
plot(phi_r*180/pi, camb_left*180/pi,  'r--s','LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Left (',rear_conditional_string(wheel_side_r),')'])
plot(phi_r(n0_r)*180/pi, camb_right(n0_r)*180/pi, 'g*', 'MarkerSize',14, ...
    'LineWidth',2,'HandleVisibility','off')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
legend('Location','best','FontSize',8)

add_sign_box_rear(sign_block_r)

%% -----------------------------------------------------------------------
%  FIGURE 2: Toe & Camber During Heave — Both Wheels
%  (Rear equivalent of front Fig 3)
%  -----------------------------------------------------------------------
figure('Name', 'Figure 2 - Rear Heave Response  |  Both Wheels')

% --- Toe vs Heave: Right wheel (top-left) ---
axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({['Toe Change vs Heave  |  Rear-', wheel_side_r, ' Wheel']; ...
       'No steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')

tj_R = toe_right_chg(jounce_idx_r);
td_R = toe_right_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [tj_R; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [td_R; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, toe_right_chg, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Toe change — Right (',wheel_side_r,')'])
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

% --- Camber vs Heave: Right wheel (top-right) ---
axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Rear-', wheel_side_r, ' Wheel']; ...
       'No steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')

cj_R = camb_right_chg(jounce_idx_r);
cd_R = camb_right_chg(droop_idx_r);
fill([hj_r; hj_r(end); hj_r(1)], [cj_R; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd_r; hd_r(end); hd_r(1)], [cd_R; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave_r, camb_right_chg, 'r-s', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName',['Camber change — Right (',wheel_side_r,')'])
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

% --- Left Wheel: Toe vs Heave (bottom-left) ---
axes('position',[0.06, 0.14, 0.38, 0.33])
hold on, grid on
title({['Toe Change vs Heave  |  Rear-', rear_conditional_string(wheel_side_r), ' Wheel']; ...
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
    'DisplayName',['Toe change — Left (',rear_conditional_string(wheel_side_r),')'])
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

% --- Left Wheel: Camber vs Heave (bottom-right) ---
axes('position',[0.56, 0.14, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Rear-', rear_conditional_string(wheel_side_r), ' Wheel']; ...
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
    'DisplayName',['Camber change — Left (',rear_conditional_string(wheel_side_r),')'])
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
%  (Rear equivalent of front Fig 4 correlation panel)
%  -----------------------------------------------------------------------
figure('Name', 'Figure 3 - Rear Toe-Camber Correlation & Summary')

% Toe-Camber Correlation: Right (top-left)
axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Toe-Camber Correlation During Heave'; ...
       ['Rear-', wheel_side_r, ' Wheel  |  Coloured by heave [mm]']}, ...
    'FontWeight','bold')
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

% Toe-Camber Correlation: Left (top-right)
axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({'Toe-Camber Correlation During Heave'; ...
       ['Rear-', rear_conditional_string(wheel_side_r), ' Wheel  |  Coloured by heave [mm]']}, ...
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

% Summary Table (bottom, centred)
axes('position',[0.15, 0.10, 0.70, 0.38])
axis off
title({'Rear Axle Kinematics Summary  |  Heave Motion Only'}, 'FontWeight','bold')

% Build summary table text
n_rows_r = 9;
key_idx_r = round(linspace(1, n_r, n_rows_r));

col_phi  = phi_r(key_idx_r)  * 180/pi;
col_hv   = heave_r(key_idx_r);
col_toeR = toe_right_chg(key_idx_r);
col_camR = camb_right_chg(key_idx_r);
col_toeL = toe_left_chg(key_idx_r);
col_camL = camb_left_chg(key_idx_r);

headers_r = {'\phi [deg]','Heave [mm]','\DeltaToe_R [deg]','\DeltaCamb_R [deg]', ...
             '\DeltaToe_L [deg]','\DeltaCamb_L [deg]'};
col_x_r   = [0.04, 0.19, 0.36, 0.52, 0.68, 0.84];
row_y_r   = 0.92;
row_dy_r  = 0.095;

for c = 1:length(headers_r)
    text(col_x_r(c), row_y_r, headers_r{c}, ...
        'FontSize',8,'FontWeight','bold', ...
        'HorizontalAlignment','center','Interpreter','tex')
end
annotation('line',[0.15, 0.85],[0.435, 0.435],'Color','k','LineWidth',1.5)

for r = 1:length(key_idx_r)
    ry_r = row_y_r - r * row_dy_r;
    vals_r = {sprintf('%.3f', col_phi(r)), ...
              sprintf('%.2f',  col_hv(r)), ...
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
fprintf('REAR AXLE KINEMATICS — VALIDATION SUMMARY\n')
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

%% -----------------------------------------------------------------------
%  HELPER FUNCTION: Opposite wheel side string
%% -----------------------------------------------------------------------
function opposite = rear_conditional_string(wheel_side)
    if strcmp(wheel_side, 'Right')
        opposite = 'Left';
    else
        opposite = 'Right';
    end
end

%% -----------------------------------------------------------------------
%  HELPER FUNCTION: Sign convention ribbon for rear figures
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
