%% =========================================================================
%  Suspension Kinematics of Double Wishbone Rack and Pinion for Front Axle
%  =========================================================================
clear; clc; close all
paramR26;

%% =========================================================================
%  >>>  USER PARAMETERS — EDIT HERE  <<<
%  =========================================================================

% -------------------------------------------------------------------------
%  1. SUSPENSION TRAVEL LIMITS
% -------------------------------------------------------------------------
par.umx          = 0.08255 / 2;        % [m]   max. rack displacement (half total)
par.phmx_droop   = 2.02  / 180 * pi;  % [rad] max lower arm rotation — droop/extension
par.phmx_jounce  = 5.16  / 180 * pi;  % [rad] max lower arm rotation — jounce/compression

% -------------------------------------------------------------------------
%  2. HARDPOINT COORDINATES  [x; y; z]  in metres  (ISO 8855: X fwd, Y left, Z up)
%     Negative Y => RIGHT-hand side wheel  (analyzed wheel)
% -------------------------------------------------------------------------
par.rvwk = [0.89604;  -0.605;      0.2032    ];  % W  wheel rim centre
par.rvak = [0.834;    -0.201;      0.104816  ];  % A  lower arm @ chassis rear
par.rvbk = [1.04;     -0.201;      0.104816  ];  % B  lower arm @ chassis front
par.rvck = [0.90186;  -0.5583;     0.121266  ];  % C  lower arm @ wheel body
par.rvdk = [0.8316;   -0.2819;     0.2420437 ];  % D  upper arm @ chassis rear
par.rvek = [1.002;    -0.2819;     0.2420437 ];  % E  upper arm @ chassis front
par.rvfk = [0.88988;  -0.5341618;  0.2926    ];  % F  upper arm @ wheel body
par.rvrk = [0.8404;   -0.21368;    0.164     ];  % R  drag link @ rack
par.rvqk = [0.80924;  -0.49872;    0.1982    ];  % Q  drag link @ wheel body

% -------------------------------------------------------------------------
%  3. WHEEL / TYRE INITIAL CONDITIONS
% -------------------------------------------------------------------------
toe0  = 0 / 180 * pi;   % [rad]  initial toe angle   (ISO 8855, + = toe-in)
camb0 = 0 / 180 * pi;   % [rad]  initial camber angle (ISO 8855, + = top outboard)
rs    = car.r_wheel;         % [m]    steady-state tyre radius

% -------------------------------------------------------------------------
%  4. VEHICLE GEOMETRY FOR ACKERMANN CALCULATION
% -------------------------------------------------------------------------
wheelbase    = car.wheelbase;   % [m]   vehicle wheelbase  (a)
% track width derived automatically from hardpoints:
%   s = 2 * |par.rvwk(2)|   (see Figure 5 computation block)

% -------------------------------------------------------------------------
%  5. STEERING RACK — C-FACTOR
%     Definition: rack travel per one full revolution of the steering wheel
%     Sign: negative rack displacement (-u) => left steer (positive SW angle)
% -------------------------------------------------------------------------
c_factor_in_per_rev = 4.71;   % [inch / rev]  c-factor of steering rack

% -------------------------------------------------------------------------
%  6. DISCRETIZATION
% -------------------------------------------------------------------------
n_droop  = 10;    % number of droop  steps (including design position)
n_jounce = 15;    % number of jounce steps (including design position)
m        = 20;   % number of rack displacement steps

% -----------------------------------------------------------------------
%  RACK DISPLACEMENT DISCRETIZATION
%  -----------------------------------------------------------------------
u = linspace(-par.umx, par.umx, m);  % [m]  symmetric rack travel

% -------------------------------------------------------------------------
%  7. WHEEL IDENTIFICATION & SIGN CONVENTION
%     Analyzed wheel hardpoints (negative Y = RIGHT wheel)
%     Left wheel will be computed by mirroring Y-coordinates
% -------------------------------------------------------------------------
wheel_side = 'Right';    % 'Right' or 'Left'  — determines analyzed wheel and hardpoints
conv_str   = 'ISO 8855';

%% =========================================================================
%  END OF USER PARAMETERS
%  Do not edit below this line unless modifying the analysis logic
%% =========================================================================

%% -----------------------------------------------------------------------
%  DERIVED QUANTITIES FROM PARAMETERS (for analyzed wheel)
%  -----------------------------------------------------------------------
en0 = [0; 0; 1];   % road normal (flat horizontal road)

% Wheel / tyre orientation unit vectors at design position (analyzed wheel)
eyrk = [toe0; 1; -camb0];  eyrk = eyrk / norm(eyrk);  % wheel rotation axis
exk  = cross(eyrk, en0);   exk  = exk  / norm(exk);    % longitudinal direction
eyk  = cross(en0,  exk);                                % lateral direction
ezk  = cross(exk,  eyrk);                               % radial direction
rwpk = -rs * ezk;                                       % W --> P (contact point)

% Kingpin inclination (sigma) and caster angle (nu) at design position
rcfk = par.rvfk - par.rvck;
ecfk = rcfk / norm(rcfk);
si   = atan2(-ecfk(2), ecfk(3));
nu   = atan2(-ecfk(1), ecfk(3));
disp(['sigma = ', num2str(si * 180/pi), ' deg'])
disp(['nue   = ', num2str(nu * 180/pi), ' deg'])

% Caster offset and scrub radius at design position
rcpk = par.rvwk + rwpk - par.rvck;
rsck = -(en0.' * rcpk) / (en0.' * ecfk) * ecfk;
co   = -exk' * (rsck + rcpk);
sr_geom = eyk' * (rsck + rcpk);
disp(['caster offset = ', num2str(co)])
disp(['scrub radius  = ', num2str(sr_geom)])

%% -----------------------------------------------------------------------
%  WHEEL IDENTIFICATION AND SIGN CONVENTION ANNOTATION BLOCK (ISO 8855)
%  -----------------------------------------------------------------------
% Y-coordinate of wheel centre is negative => RIGHT-HAND side wheel (analyzed)
% ISO 8855 coordinate system:
%   X  : forward (longitudinal)
%   Y  : leftward (lateral)  => negative Y means RIGHT wheel
%   Z  : upward  (vertical)
%
% Angles (ISO 8855 / SAE J670 hybrid, consistent throughout):
%   Toe    (+) : toe-in     (wheel rotates toward vehicle centreline)
%   Camber (+) : top outboard
%   Phi    (+) : jounce / compression  (lower arm rotates upward)
%   Phi    (-) : droop  / extension    (lower arm rotates downward)
%   u      (+) : rack displacement positive => RIGHT steer
%   u      (-) : rack displacement negative => LEFT steer
%
% Steering wheel angle: positive = LEFT turn

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

%% -----------------------------------------------------------------------
%  ASYMMETRICAL SUSPENSION TRAVEL DISCRETIZATION
%  -----------------------------------------------------------------------
n = n_droop + n_jounce - 1;

phi_droop  = linspace(-par.phmx_droop,  0, n_droop);
phi_jounce = linspace(0, par.phmx_jounce, n_jounce);
phi        = [phi_droop(1:end-1), phi_jounce];

% Find design position (phi closest to 0)
[~, n0] = min(abs(phi));

disp('Suspension travel discretization:')
disp(['  Droop:  ', num2str(n_droop),  ' steps, ', ...
      num2str(-par.phmx_droop  * 180/pi), ' deg'])
disp(['  Jounce: ', num2str(n_jounce), ' steps, +', ...
      num2str( par.phmx_jounce * 180/pi), ' deg'])
disp(['  Total:  ', num2str(n), ' steps'])
disp(['  Design position at index n0 = ', num2str(n0), ' (phi = ', ...
      num2str(phi(n0)*180/pi, '%.3f'), ' deg)'])


%% -----------------------------------------------------------------------
%  COMPUTE SUSPENSION KINEMATICS FOR ANALYZED WHEEL (paths, toe, camber for Fig 1,3,4)
%  -----------------------------------------------------------------------
xw = zeros(n,m); yw = xw; zw = xw;
xp = xw;         yp = xw; zp = xw;
del_analyzed = xw; toe_analyzed = xw; camb_analyzed = xw; ddel = xw;  % Renamed for clarity

for i = 1:n
    for j = 1:m
        [avw, rvwv, del_analyzed(i,j), pd] = fun_05_dblwb_kin(phi(i), u(j), par);
        eyrv       = avw * eyrk;
        rvpv       = rvwv + avw * rwpk;
        xw(i,j)    = rvwv(1);  yw(i,j) = rvwv(2);  zw(i,j) = rvwv(3);
        xp(i,j)    = rvpv(1);  yp(i,j) = rvpv(2);  zp(i,j) = rvpv(3);
        toe_analyzed(i,j)   = atan2(-eyrv(1),  eyrv(2));
        camb_analyzed(i,j)  = atan2( eyrv(3),  eyrv(2));
        ddel(i,j)  = norm(pd(:,2));
    end
end

[~, n0] = min(abs(phi));   % design position index (phi closest to 0)
m0      = round(m / 2);    % neutral steering index (u = 0)
rvpk    = par.rvwk + rwpk;

%% -----------------------------------------------------------------------
%  COMPUTE STEER ANGLES FOR BOTH WHEELS AT DESIGN HEIGHT (for Fig 2,5: actual left/right, no mirroring)
%  -----------------------------------------------------------------------
del_right = zeros(1, m);
del_left  = zeros(1, m);

% Analyzed wheel steer (already computed at design height)
if strcmp(wheel_side, 'Right')
    del_right = del_analyzed(n0, :);
    % Mirror hardpoints to create left wheel geometry
    par_opposite = par;
    fields = {'rvwk', 'rvak', 'rvbk', 'rvck', 'rvdk', 'rvek', 'rvfk', 'rvrk', 'rvqk'};
    for f = 1:length(fields)
        par_opposite.(fields{f})(2) = -par.(fields{f})(2);  % Mirror Y (negative to positive)
    end
    % Compute left wheel steer at design height
    for j = 1:m
        [~, ~, del_left(j), ~] = fun_05_dblwb_kin(phi(n0), u(j), par_opposite);
    end
else  % If analyzing left, swap roles (symmetric code not implemented here, assume Right)
    error('Script assumes wheel_side = ''Right'' for mirroring logic. Adjust if needed.');
end

%% -----------------------------------------------------------------------
%  STEERING WHEEL ANGLE CALCULATION (convention: -u => left steer => positive SW)
%  -----------------------------------------------------------------------
u_inch = u * 39.3701;  % [m] -> [inch]
sw_angle_deg = - (u_inch / c_factor_in_per_rev) * 360;  % [deg]  Negative u -> positive SW (left turn)

%% -----------------------------------------------------------------------
%  DIAGNOSTIC: CHECK NEUTRAL POSITION OFFSETS (at exact φ=0, u=0)
%  -----------------------------------------------------------------------

u_neutral = 0;    % [m] Exact rack displacement at neutral
phi_neutral = 0;  % [rad] Exact roll/bump angle at neutral

% Compute RIGHT wheel at neutral (use original par)
[~, ~, delta_R_neut, ~] = fun_05_dblwb_kin(phi_neutral, u_neutral, par);

% Mirror for LEFT wheel
par_opposite = par;
fields = {'rvwk', 'rvak', 'rvbk', 'rvck', 'rvdk', 'rvek', 'rvfk', 'rvrk', 'rvqk'};
for f = 1:length(fields)
    par_opposite.(fields{f})(2) = -par.(fields{f})(2);  % Mirror Y
end
[~, ~, delta_L_neut, ~] = fun_05_dblwb_kin(phi_neutral, u_neutral, par_opposite);

% Camber: Extract from orientation (eyrv computation)
eyrk_right = [toe0; 1; -camb0];  eyrk_right = eyrk_right / norm(eyrk_right);
[avw_right, ~, ~, ~] = fun_05_dblwb_kin(phi_neutral, u_neutral, par);
eyrv_right = avw_right * eyrk_right;
camber_R_neut = atan2(eyrv_right(3), eyrv_right(2)) * 180/pi;

eyrk_left = [toe0; 1; -camb0];  eyrk_left = eyrk_left / norm(eyrk_left);  % Same initial
[avw_left, ~, ~, ~] = fun_05_dblwb_kin(phi_neutral, u_neutral, par_opposite);
eyrv_left = avw_left * eyrk_left;
camber_L_neut = atan2(eyrv_left(3), eyrv_left(2)) * 180/pi;  % Note: may need sign flip for left camber convention

fprintf('\n===== NEUTRAL POSITION ANALYSIS (exact u=0, φ=0) =====\n')
fprintf('Steer Angle - Left:   %.3f°\n', delta_L_neut * 180/pi)
fprintf('Steer Angle - Right:  %.3f°\n', delta_R_neut * 180/pi)
fprintf('Steer Angle - Mean:   %.3f°\n', mean([delta_L_neut * 180/pi, delta_R_neut * 180/pi]))
fprintf('\nCamber Angle - Left:  %.3f°\n', camber_L_neut)
fprintf('Camber Angle - Right: %.3f°\n', camber_R_neut)
fprintf('Camber Angle - Mean:  %.3f°\n', mean([camber_L_neut, camber_R_neut]))

% Warnings
if abs((delta_L_neut - delta_R_neut) * 180/pi) > 0.5
    fprintf('\n⚠️  WARNING: Large steer asymmetry at neutral!\n')
    fprintf('   Difference: %.3f°\n', abs((delta_L_neut - delta_R_neut) * 180/pi))
end

if abs(camber_L_neut - camber_R_neut) > 0.3
    fprintf('\n⚠️  WARNING: Large camber asymmetry at neutral!\n')
    fprintf('   Difference: %.3f°\n', abs(camber_L_neut - camber_R_neut))
end

% Expected: If toe0=0 and symmetric, both should be ~0°. Offsets indicate geometry bias.
%% -----------------------------------------------------------------------
%  HELPER FUNCTION: Determine opposite wheel side
% -----------------------------------------------------------------------
function opposite = conditional_string(wheel_side)
if strcmp(wheel_side, 'Right')
    opposite = 'Left';
else
    opposite = 'Right';
end
end

%% -----------------------------------------------------------------------
%  HELPER FUNCTION: add sign-convention ribbon to any figure
%  -----------------------------------------------------------------------
function add_sign_box(sign_block, wheel_side)
    annotation('textbox', [0.00, 0.00, 1.00, 0.10], ...
        'String',          sign_block, ...
        'FontSize',        7.5, ...
        'FontName',        'Courier New', ...
        'BackgroundColor', [0.95 0.95 0.85], ...
        'EdgeColor',       [0.40 0.40 0.20], ...
        'FitBoxToText',    'off', ...
        'LineWidth',       1, ...
        'Interpreter',     'none', ...
        'VerticalAlignment','middle');
end

%% -----------------------------------------------------------------------
%  FIGURE 1: Wheel Centre Paths (for analyzed wheel)
%  -----------------------------------------------------------------------
figure('Name', ['Figure 1 - Wheel Center Paths  |  Analyzed: ', wheel_side, ' Wheel'])

% XZ Plane
axes('position',[0.05, 0.12, 0.18, 0.78])
hold on, axis equal, grid on
title({'Longitudinal Plane (XZ)'; ['Front-', wheel_side]}, 'FontWeight','bold')
xlabel('x  [m]  (+ forward)')
ylabel('z  [m]  (+ upward)')
h1 = plot(xw(:,m0), zw(:,m0), 'b-',  'LineWidth',1.5);
h2 = plot(xp(:,m0), zp(:,m0), 'b--', 'LineWidth',1.5);
h3 = plot(par.rvwk(1), par.rvwk(3), 'ok', 'MarkerFaceColor','k',    'MarkerSize',6);
h4 = plot(rvpk(1),     rvpk(3),     'o',  'Color','cyan', ...
          'MarkerFaceColor','cyan', 'MarkerSize',6);
legend([h1,h2,h3,h4], 'W center','Contact P','W design','P design', ...
    'Location','best','FontSize',8)

% YZ Plane
axes('position',[0.28, 0.12, 0.18, 0.78])
hold on, axis equal, grid on
title({'Lateral Plane (YZ)'; ['Front-', wheel_side]}, 'FontWeight','bold')
xlabel({'y  [m]'; '(+left, -Y=right)'})
ylabel('z  [m]  (+ upward)')
h1 = plot(yw(:,m0), zw(:,m0), 'r-',  'LineWidth',1.5);
h2 = plot(yp(:,m0), zp(:,m0), 'r--', 'LineWidth',1.5);
h3 = plot(par.rvwk(2), par.rvwk(3), 'ok', 'MarkerFaceColor','k',    'MarkerSize',6);
h4 = plot(rvpk(2),     rvpk(3),     'o',  'Color','cyan', ...
          'MarkerFaceColor','cyan', 'MarkerSize',6);
legend([h1,h2,h3,h4], 'W center','Contact P','W design','P design', ...
    'Location','best','FontSize',8)

% Toe Surface Map (top-right) — analyzed wheel
axes('position',[0.55, 0.55, 0.42, 0.35])
surf(u*1000, phi*180/pi, toe_analyzed*180/pi), colormap('parula')
grid on, view(-40,10)
title({'Toe Angle Map'; '(+) = toe-in'}, 'FontWeight','bold')
xlabel('u  [mm]  (+: right steer)')
ylabel('\phi [deg] (+:jounce/-:droop)')
zlabel('Toe [deg] (+: toe-in)')

% Camber Surface Map (bottom-right) — analyzed wheel
axes('position',[0.55, 0.12, 0.42, 0.35])
surf(u*1000, phi*180/pi, camb_analyzed*180/pi), colormap('parula')
grid on, view(-40,10)
title({'Camber Angle Map'; '(+) = top outboard'}, 'FontWeight','bold')
xlabel('u  [mm]  (+: right steer)')
ylabel('\phi [deg] (+:jounce/-:droop)')
zlabel('Camber [deg] (+: top outboard)')

add_sign_box(sign_block, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 2: Steering Kinematics (both wheels, actual geometry)
%  -----------------------------------------------------------------------
figure('Name', ['Figure 2 - Steering Kinematics  |  Analyzed: ', wheel_side, ' Wheel'])

d1  = del_right;  % Analyzed wheel (right)
d2  = del_left;   % Opposite wheel (left) — actual computation, no sign flip
a   = wheelbase;
s   = 2 * abs(par.rvwk(2));  % Track width (positive)
% Ideal Ackermann (corrected formula: a - s * tan for inner)
d2a = atan2(a * tan(d1), a - s * tan(d1));  % Example for d1 outer; full logic in Fig5

% Steering Angles (top-left)
axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Steering Angles vs Rack Displacement'; ...
       ['Front Axle  |  Design ride height']}, 'FontWeight','bold')
xlabel('Rack disp.  u  [mm]  (+: right steer)')
ylabel('Steer angle  \delta  [deg]  (+: left steer)')
plot(u*1000, d1*180/pi, 'b-',  'LineWidth',1.5, 'DisplayName','Right wheel  \delta_{right}')
plot(u*1000, d2*180/pi, 'r--', 'LineWidth',1.5, 'DisplayName','Left wheel  \delta_{left}')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

% Steering Sensitivity (bottom-left) — analyzed wheel
axes('position',[0.06, 0.14, 0.38, 0.33])
hold on, grid on
title({'Steering Sensitivity  d\delta/du'; ...
       '(+u): right steer  |  Design height'}, 'FontWeight','bold')
xlabel('Rack disp.  u  [mm]  (+: right steer)')
ylabel('d\delta / du  [rad/m]')
plot(u*1000, ddel(n0,:), 'm-', 'LineWidth',1.5, 'DisplayName','d\delta/du (analyzed wheel)')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

% Ackermann (right block — full height)
axes('position',[0.54, 0.14, 0.43, 0.77])
hold on, axis equal, grid on
title({'Ackermann Comparison'; ...
       '\delta_{outer} vs \delta_{inner} (both wheels)'}, 'FontWeight','bold')
xlabel('\delta_{outer}  [deg]  (+: left steer)')
ylabel('\delta_{inner}  [deg]  (+: left steer)')
plot(d1*180/pi, d2*180/pi,  'b-',  'LineWidth',2.0, 'DisplayName','Kinematic (right vs left)')
plot(d1*180/pi, d2a*180/pi, 'r--', 'LineWidth',1.5, ...
    'DisplayName', ['Ideal Ackermann (a=', num2str(a), 'm)'])
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

add_sign_box(sign_block, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 3: Toe and Camber During Heave (analyzed wheel)
%  -----------------------------------------------------------------------
figure('Name', ['Figure 3 - Heave Response  |  Analyzed: ', wheel_side, ' Wheel'])

z_design   = zw(n0, m0);
heave      = (zw(:, m0) - z_design) * 1000;
toe_heave  = (toe_analyzed(:, m0)  - toe_analyzed(n0,  m0)) * 180/pi;
camb_heave = (camb_analyzed(:, m0) - camb_analyzed(n0, m0)) * 180/pi;

jounce_idx = find(heave >= 0);
droop_idx  = find(heave <= 0);

hj = heave(jounce_idx);   tj = toe_heave(jounce_idx);
hd = heave(droop_idx);    td = toe_heave(droop_idx);

% Toe vs Heave (top-left)
axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({['Toe Change vs Heave  |  Front-', wheel_side, ' Wheel']; ...
       'Neutral steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
fill([hj; hj(end); hj(1)], [tj; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd; hd(end); hd(1)], [td; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave, toe_heave, 'b-o', 'LineWidth',2, 'MarkerSize',5, 'DisplayName','Toe change')
plot(heave(n0), toe_heave(n0), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
text(max(heave)*0.1, max(abs(toe_heave))*0.75, 'JOUNCE', ...
    'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(heave)*0.9, max(abs(toe_heave))*0.75, 'DROOP', ...
    'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
legend('Location','southwest','FontSize',9)

% Camber vs Heave (top-right)
axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Heave  |  Front-', wheel_side, ' Wheel']; ...
       'Neutral steering  (u = 0)'}, 'FontWeight','bold')
xlabel('Heave  h  [mm]  (+: jounce)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')

cj      = camb_heave(jounce_idx);
cd_vals = camb_heave(droop_idx);
fill([hj; hj(end); hj(1)], [cj; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([hd; hd(end); hd(1)], [cd_vals; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(heave, camb_heave, 'r-s', 'LineWidth',2, 'MarkerSize',5, 'DisplayName','Camber change')
plot(heave(n0), camb_heave(n0), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Design pos.')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
text(max(heave)*0.1, max(abs(camb_heave))*0.75, 'JOUNCE', ...
    'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(heave)*0.9, max(abs(camb_heave))*0.75, 'DROOP', ...
    'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
legend('Location','southwest','FontSize',9)

% Toe-Camber Correlation (bottom, centred)
axes('position',[0.20, 0.14, 0.55, 0.33])
hold on, grid on
title({'Toe-Camber Correlation During Heave'; ...
       ['Front-', wheel_side, ' Wheel  |  Coloured by heave [mm]']}, 'FontWeight','bold')
xlabel('\Delta\delta_{toe}  [deg]  (+: toe-in)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
scatter(toe_heave, camb_heave, 80, heave, 'filled','o','DisplayName','Heave steps')
cbar = colorbar;
cbar.Label.String  = 'Heave  h  [mm]  (+: jounce)';
cbar.Label.FontSize = 9;
plot(toe_heave(n0), camb_heave(n0), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Design pos.')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

add_sign_box(sign_block, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 4: Steer Angle and Camber During Steering (analyzed wheel)
%  -----------------------------------------------------------------------
figure('Name', ['Figure 4 - Steering Response  |  Analyzed: ', wheel_side, ' Wheel'])

% AMENDED: Use actual steer angle del_analyzed instead of toe_analyzed
steer_angle  = (del_analyzed(n0, :) - del_analyzed(n0, m0)) * 180/pi;
camb_steer   = (camb_analyzed(n0, :) - camb_analyzed(n0, m0)) * 180/pi;

u_col           = u(:);
steer_angle_col = steer_angle(:);
camb_steer_col  = camb_steer(:);

right_idx = find(u_col >= 0);  % +u: right steer
left_idx  = find(u_col <= 0);  % -u: left steer

ur = u_col(right_idx) * 1000;
ul = u_col(left_idx)  * 1000;

% ===== FIND u=0 INDEX =====
[~, u0_idx] = min(abs(u));  % Index closest to u=0

% Steer Angle vs Steering Input (top-left)
axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({['Steer Angle vs Rack Displacement  |  Front-', wheel_side, ' Wheel']; ...
       'Design ride height  (\phi = 0)'}, 'FontWeight','bold')
xlabel('Rack disp.  u  [mm]  (+: right steer)')
ylabel('\delta  [deg]  (+: left steer)')

sr = steer_angle_col(right_idx);
sl = steer_angle_col(left_idx);

fill([ur; ur(end); ur(1)], [sr; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([ul; ul(end); ul(1)], [sl; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(u*1000, steer_angle, 'b-o', 'LineWidth',2, 'MarkerSize',5, ...
    'DisplayName','Steer angle  \delta')
% ===== GREEN STAR AT u=0 =====
plot(u(u0_idx)*1000, steer_angle(u0_idx), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Neutral (u=0)')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
text(max(u)*1000*0.1,  max(abs(steer_angle))*0.75, 'STEER RIGHT', ...
    'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(u)*1000*0.9, max(abs(steer_angle))*0.75, 'STEER LEFT', ...
    'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
legend('Location','southwest','FontSize',9)

% Camber vs Steering (top-right)
axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({['Camber Change vs Steering  |  Front-', wheel_side, ' Wheel']; ...
       'Design ride height  (\phi = 0)'}, 'FontWeight','bold')
xlabel('Rack disp.  u  [mm]  (+: right steer)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')

cr = camb_steer_col(right_idx);
cl = camb_steer_col(left_idx);
fill([ur; ur(end); ur(1)], [cr; 0; 0], ...
    [1 0.85 0.85],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
fill([ul; ul(end); ul(1)], [cl; 0; 0], ...
    [0.85 0.90 1],'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off')
plot(u*1000, camb_steer, 'r-s', 'LineWidth',2, 'MarkerSize',5, 'DisplayName','Camber change')
% ===== GREEN STAR AT u=0 =====
plot(u(u0_idx)*1000, camb_steer(u0_idx), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Neutral (u=0)')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
text(max(u)*1000*0.1,  max(abs(camb_steer))*0.75, 'STEER RIGHT', ...
    'Color',[0.7 0 0],'FontSize',8,'FontWeight','bold')
text(min(u)*1000*0.9, max(abs(camb_steer))*0.75, 'STEER LEFT', ...
    'Color',[0 0 0.7],'FontSize',8,'FontWeight','bold')
legend('Location','northwest','FontSize',9)

% Steer-Camber Correlation (bottom, centred)
axes('position',[0.20, 0.14, 0.55, 0.33])
hold on, grid on
title({'Steer-Camber Correlation During Steering'; ...
       ['Front-', wheel_side, ' Wheel  |  Coloured by rack disp. [mm]']}, 'FontWeight','bold')
xlabel('\delta  [deg]  (+: left steer)')
ylabel('\Delta\gamma  [deg]  (+: top outboard)')
scatter(steer_angle, camb_steer, 80, u*1000, 'filled','o','DisplayName','Steering steps')
cbar = colorbar;
cbar.Label.String  = 'Rack disp.  u  [mm]  (+: right steer)';
cbar.Label.FontSize = 9;
% ===== GREEN STAR AT u=0 =====
plot(steer_angle(u0_idx), camb_steer(u0_idx), 'g*', 'MarkerSize',14, 'LineWidth',2, ...
    'DisplayName','Neutral (u=0)')
legend('Location','best','FontSize',9)
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')

add_sign_box(sign_block, wheel_side)

%% -----------------------------------------------------------------------
%  FIGURE 5: Steering Ratio and Ackermann Analysis (both wheels)
%  -----------------------------------------------------------------------
figure('Name', ['Figure 5 - Steering Ratio & Ackermann  |  Analyzed: ', wheel_side, ' Wheel'])

d1_deg = del_right * 180/pi;  % Right wheel (analyzed)
d2_deg = del_left  * 180/pi;  % Left wheel (opposite)

% Validity check: avoid division by near-zero angles
valid = abs(d1_deg) > 0.1;  % Only use data where analyzed wheel angle > 0.1°

% Guard: replace near-zero wheel angles with NaN
d1_safe = d1_deg;
d2_safe = d2_deg;
d1_safe(~valid) = NaN;
d2_safe(~valid) = NaN;

% Calculate steering ratio
SR_right   = sw_angle_deg ./ d1_safe;  % SR for right (analyzed) wheel
SR_left    = sw_angle_deg ./ d2_safe;  % SR for left (opposite) wheel

%% -----------------------------------------------------------------------
%  ACKERMANN GEOMETRY — Corrected cotangent formula
%  Reference: Ackermann steering condition
%  cot(d_outer) - cot(d_inner) = s/a  (full track / wheelbase)
%  => ideal_inner = arccot( cot(outer) - s/a )
%% -----------------------------------------------------------------------
a = wheelbase;
s = 2 * abs(par.rvwk(2));   % full track width [m]

fprintf('\n===== ACKERMANN PARAMETERS =====\n')
fprintf('Wheelbase  a: %.4f m = %.1f mm\n', a, a*1000)
fprintf('Track width s: %.4f m = %.1f mm\n', s, s*1000)
fprintf('s/a ratio: %.4f\n\n', s/a)

pct_ackermann = zeros(1, m);
d2a_signed    = zeros(1, m);

valid_ack = abs(d1_deg) > 0.1;   % exclude near-zero steer only

%% REVISED ACKERMANN WITH MINIMUM THRESHOLD
min_steer_threshold = 5.0;  % degrees — below this, % Ackermann is unreliable

for k = 1:m
    % Identify outer and inner
    if abs(d1_deg(k)) <= abs(d2_deg(k))
        d_outer_abs        = abs(d1_deg(k));
        d_inner_actual_abs = abs(d2_deg(k));
        sign_outer         = sign(d1_deg(k));
    else
        d_outer_abs        = abs(d2_deg(k));
        d_inner_actual_abs = abs(d1_deg(k));
        sign_outer         = sign(d2_deg(k));
    end

    % ← NEW: Skip low angles
    if d_outer_abs < min_steer_threshold
        pct_ackermann(k) = NaN;
        d2a_signed(k)    = NaN;
        continue
    end

    if d_outer_abs < 0.05
        pct_ackermann(k) = NaN;
        d2a_signed(k)    = NaN;
        continue
    end

    % Ackermann calculation (unchanged)
    cot_outer       = 1 / tan(d_outer_abs * pi/180);
    cot_ideal_inner = cot_outer - s/a;

    if cot_ideal_inner <= 0
        pct_ackermann(k) = NaN;
        d2a_signed(k)    = NaN;
        continue
    end

    ideal_inner_abs = atan(1 / cot_ideal_inner) * 180/pi;
    d2a_signed(k)   = sign_outer * ideal_inner_abs;

    actual_spread = d_inner_actual_abs - d_outer_abs;
    ideal_spread  = ideal_inner_abs    - d_outer_abs;

    if ideal_spread > 0.001
        pct_ackermann(k) = (actual_spread / ideal_spread) * 100;
    else
        pct_ackermann(k) = NaN;
    end
end

fprintf('\n===== ACKERMANN DIAGNOSTIC =====\n')
fprintf('Min steering threshold: %.1f°\n', min_steer_threshold)
valid_pts = ~isnan(pct_ackermann);
fprintf('Valid points above threshold: %d / %d\n', sum(valid_pts), m)

% Bottom ribbon for Figure 5
opposite_side = 'Left';
sr_block = {
    ['Analyzed: Front-', wheel_side, ...
     '  |  Opposite: Front-', opposite_side, ...
     '  |  C-factor = ', num2str(c_factor_in_per_rev), ' in/rev'], ...
    ['Track width: s = ', num2str(s*1000,'%.1f'), ...
     ' mm  |  Wheelbase: a = ', num2str(a*1000,'%.0f'), ' mm'], ...
    'Steering Ratio = SW angle / road wheel angle  (+ SW: left steer)', ...
    '% Ackermann: 0% = parallel steer,  100% = ideal Ackermann,  >100% = over-Ackermann'
};

% Plot 5a: Steering Ratio vs SW Angle (top-left)
axes('position',[0.06, 0.58, 0.38, 0.33])
hold on, grid on
title({'Steering Ratio vs Steering Wheel Angle'; ...
       ['Front Axle  |  Design ride height  (\phi = 0)']}, ...
    'FontWeight','bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)')
ylabel('Steering ratio  SR  [-]')
plot(sw_angle_deg(valid), SR_right(valid), 'b-o', 'LineWidth',2, ...
    'MarkerSize',5, 'DisplayName', ['SR Right wheel (', wheel_side, ')'])
plot(sw_angle_deg(valid), SR_left(valid),  'r--s','LineWidth',2, ...
    'MarkerSize',5, 'DisplayName',['SR ', opposite_side, ' wheel'])
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
[~, first_v] = find(valid,1);  % First valid point
if ~isempty(first_v)
    text(sw_angle_deg(first_v)*1.05, SR_right(first_v), ...
        sprintf('SR_{right} = %.1f', SR_right(first_v)), ...
        'FontSize',8,'Color','blue','FontWeight','bold')
end
legend('Location','best','FontSize',9)

% Plot 5b: Road Wheel Angles vs SW Angle (top-right)
axes('position',[0.56, 0.58, 0.38, 0.33])
hold on, grid on
title({'Road Wheel Steer Angles vs Steering Wheel Angle'; ...
       ['Front Axle  |  Design ride height  (\phi = 0)']}, ...
    'FontWeight','bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)')
ylabel('Road wheel angle  \delta  [deg]  (+: left steer)')
plot(sw_angle_deg, d1_deg,      'b-',  'LineWidth', 2.0, ...
    'DisplayName', ['Right wheel  \delta_{right} (', wheel_side, ')'])
plot(sw_angle_deg, d2_deg,      'r--', 'LineWidth', 2.0, ...
    'DisplayName', ['Left wheel  \delta_{left} (', opposite_side, ')'])
plot(sw_angle_deg, d2a_signed,  'k:',  'LineWidth', 1.5, ...
    'DisplayName', 'Ideal inner wheel  \delta_{inner,ack}')
yline(0,'k:','HandleVisibility','off')
xline(0,'k:','HandleVisibility','off')
legend('Location','best','FontSize',9)

% Plot 5c: % Ackermann vs Steering Wheel Angle (bottom-left)
axes('position',[0.06, 0.14, 0.38, 0.33])
hold on, grid on
title({'% Ackermann vs Steering Wheel Angle'; ...
       '100% = ideal Ackermann  |  0% = parallel steer'}, 'FontWeight','bold')
xlabel('Steering wheel angle  [deg]  (+: left steer)')
ylabel('% Ackermann  [%]')

plot(sw_angle_deg(valid_ack), pct_ackermann(valid_ack), 'm-o', ...
    'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', '% Ackermann')
yline(100, 'k--', 'LineWidth', 1.2, 'DisplayName', '100% (ideal Ackermann)')
yline(0,   'k:',  'HandleVisibility', 'off')
xline(0,   'k:',  'HandleVisibility', 'off')

% Shade over-Ackermann region (> 100%)
ylims = ylim;
annotation('rectangle', [0.06 + (0.68-0.06)*(100/ylims(2)), 0.68, 0.38*(1-100/ylims(2)), ylims(2)-100], ...  
    'FaceColor', [0.9 1.0 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3)  % Adjusted for position

legend('Location','best','FontSize',9)

% Plot 5d: Summary Table (bottom-right)
axes('position',[0.56, 0.14, 0.38, 0.33])
axis off
title({'Summary Table  |  Key Positions'; ...
       ['C-factor: ', num2str(c_factor_in_per_rev), ' in/rev']}, 'FontWeight','bold')

% ===== SELECT TABLE ROWS =====
n_rows = 9;  % NUMBER OF ROWS — EDIT THIS TO ADD/REMOVE ROWS
key_idx = round(linspace(1, m, n_rows));

col_sw  = sw_angle_deg(key_idx);
col_d1  = d1_deg(key_idx);
col_d2  = d2_deg(key_idx);
col_sr1 = SR_right(key_idx);
col_sr2 = SR_left(key_idx);
col_ack = pct_ackermann(key_idx);

% FIXED: Use plain text headers (no LaTeX interpreter)
headers = {'SW [deg]', '\delta_R [deg]', '\delta_L [deg]', 'SR_R', 'SR_L', '% Ack'};
col_x   = [0.05, 0.20, 0.35, 0.50, 0.65, 0.80];
row_y   = 0.92;
row_dy  = 0.095;  % Reduced for more rows

% Draw header row (fixed interpreter)
for c = 1:length(headers)
    text(col_x(c), row_y, headers{c}, ...
        'FontSize', 8, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'Interpreter', 'tex')
end

annotation('line', [0.56, 0.94], [0.54, 0.54], 'Color','k','LineWidth',1.5)

% Draw data rows
for r = 1:length(key_idx)
    ry   = row_y - r * row_dy;
    vals = {sprintf('%.1f', col_sw(r)),  ...
            sprintf('%.2f', col_d1(r)),  ...
            sprintf('%.2f', col_d2(r)),  ...
            sprintf('%.1f', col_sr1(r)), ...
            sprintf('%.1f', col_sr2(r)), ...
            sprintf('%.1f', col_ack(r))};
    if mod(r,2) == 0
        annotation('rectangle', [0.56, ry-0.04, 0.38, row_dy], ...
            'FaceColor',[0.93 0.93 0.93],'EdgeColor','none','FaceAlpha',0.6)
    end
    for c = 1:length(vals)
        text(col_x(c), ry, vals{c}, ...
            'FontSize', 8, 'HorizontalAlignment', 'center', 'Interpreter', 'none')
    end
end

add_sign_box(sr_block, wheel_side)

%% -----------------------------------------------------------------------
%  VALIDATION SUMMARY
%  -----------------------------------------------------------------------
disp('=== KINEMATIC VALIDATION ===')
disp(['Wheelbase (a):      ', num2str(wheelbase*1000, '%.1f'), ' mm'])
disp(['Track width (s):    ', num2str(s*1000, '%.1f'), ' mm'])
disp(['C-factor:          ', num2str(c_factor_in_per_rev), ' in/rev'])
disp(['Max rack travel:   ±', num2str(par.umx*1000, '%.2f'), ' mm'])
disp(['Max steering wheel angle: ±', num2str(max(abs(sw_angle_deg)), '%.1f'), ' deg'])
disp(['Ackermann range:   ', num2str(min(pct_ackermann(valid_ack)), '%.1f'), ...
    '% to ', num2str(max(pct_ackermann(valid_ack)), '%.1f'), '%'])

