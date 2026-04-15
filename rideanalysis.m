%% ============================================================
%  Pitching Ride Motion - 2-DOF Heave+Pitch, Bump Excitation
%  States: z (heave CG), z_dot, theta (pitch), theta_dot
%
%  EOM:
%    m*z_ddot     = -kf*(z - a*th - zr_f) - cf*(zdot - a*thdot - zrdot_f)
%                   -kr*(z + b*th - zr_r) - cr*(zdot + b*thdot - zrdot_r)
%
%    Iy*th_ddot   = +a*kf*(z - a*th - zr_f) + a*cf*(zdot - a*thdot - zrdot_f)
%                   -b*kr*(z + b*th - zr_r) - b*cr*(zdot + b*thdot - zrdot_r)
%
%  Axle displacements recovered as:
%    z_front = z - a*theta
%    z_rear  = z + b*theta
%% ============================================================
clear; clc; close all;
paramR26

%% ---- Vehicle Parameters ----------------------------------------
m    = car.m_sprung;            % Sprung mass              [kg]
Iy   = 212.09;           % Pitch moment of inertia  [kg*m^2]
a    = car.a;            % CG to front axle         [m]
b    = car.b;            % CG to rear  axle         [m]
L    = car.wheelbase;            % Wheelbase                [m]

kf_axle   = 2 * front.Kr;         % Front wheel/suspension stiffness [N/m]
kr_axle   = 2 * rear.Kr;          % Rear  wheel/suspension stiffness [N/m]
cf_axle   = 2 * front.cs;         % Front damping coefficient        [N*s/m]
cr_axle   = 2 * rear.cs;          % Rear  damping coefficient        [N*s/m]

%% ---- Derived Properties ----------------------------------------
% Pitch stiffness & damping for reference
K_pitch   = kf_axle*a^2 + kr_axle*b^2; % [Nm/rad]
C_pitch   = cf_axle*a^2 + cr_axle*b^2;
omega_n   = sqrt(K_pitch / Iy);
zeta      = C_pitch / (2*sqrt(K_pitch * Iy));

% Bounce natural frequency
omega_b   = sqrt((kf_axle + kr_axle) / m);
zeta_b    = (cf_axle + cr_axle) / (2*sqrt((kf_axle+kr_axle)*m));

fprintf('--- System Properties ---\n');
fprintf('Pitch  : omega_n = %.4f rad/s  |  zeta = %.4f\n', omega_n, zeta);
fprintf('Bounce : omega_n = %.4f rad/s  |  zeta = %.4f\n', omega_b, zeta_b);
if zeta < 1
    fprintf('Pitch mode: UNDERDAMPED\n\n');
else
    fprintf('Pitch mode: OVERDAMPED\n\n');
end

%% ---- Bump Profile (Versine) ------------------------------------
%  Road profile seen by each tyre: half-sine bump
%  z_road(t) = (H/2)*(1 - cos(2*pi*V*t / Lbump))  for 0 <= t <= Lbump/V
%
V_vehicle = 35.9 * 1000/3600;          % Vehicle forward speed     [m/s]
H_bump    = 0.03;        % Bump height               [m]
L_bump    = 0.2;         % Bump length               [m]

t_bump_dur = L_bump / V_vehicle;     % Duration of bump passage   [s]
t_delay_r  = L / V_vehicle;          % Rear axle time delay       [s]

% Road input functions (displacement & velocity)
bump_disp = @(t, t0) (H_bump/2) * (1 - cos(2*pi*V_vehicle*(t-t0)/L_bump)) ...
                      .* (t >= t0) .* (t <= t0 + t_bump_dur);

bump_vel  = @(t, t0) (H_bump/2) * (2*pi*V_vehicle/L_bump) ...
                      .* sin(2*pi*V_vehicle*(t-t0)/L_bump) ...
                      .* (t >= t0) .* (t <= t0 + t_bump_dur);

% Front axle: bump starts at t=0
% Rear  axle: bump starts at t=t_delay_r
zr_f    = @(t) bump_disp(t, 0);
zrdot_f = @(t) bump_vel(t,  0);
zr_r    = @(t) bump_disp(t, t_delay_r);
zrdot_r = @(t) bump_vel(t,  t_delay_r);

%% ---- Initial Conditions ----------------------------------------
% X = [z; zdot; theta; thetadot]
X0 = [0; 0; 0; 0];

%% ---- Time Integration Settings ---------------------------------
t_start = 0;
t_end   = 0.5;       % Long enough to capture transient decay   [s]
dt      = 0.001;     % Fine step for accuracy                   [s]

t_vec = t_start : dt : t_end;
N     = length(t_vec);

%% ---- Pre-Allocate ----------------------------------------------
X        = zeros(4, N);
X(:, 1)  = X0;

%% ---- RK4 Time-Stepping Loop ------------------------------------
fprintf('Running RK4 (2-DOF heave+pitch)...\n');

for i = 1 : N-1
    t_i = t_vec(i);
    X_i = X(:, i);

    k1 = heavePitchEOM(t_i,        X_i,             m, Iy, a, b, kf_axle, kr_axle, cf_axle, cr_axle, zr_f, zrdot_f, zr_r, zrdot_r);
    k2 = heavePitchEOM(t_i+dt/2,   X_i+dt/2*k1,     m, Iy, a, b, kf_axle, kr_axle, cf_axle, cr_axle, zr_f, zrdot_f, zr_r, zrdot_r);
    k3 = heavePitchEOM(t_i+dt/2,   X_i+dt/2*k2,     m, Iy, a, b, kf_axle, kr_axle, cf_axle, cr_axle, zr_f, zrdot_f, zr_r, zrdot_r);
    k4 = heavePitchEOM(t_i+dt,     X_i+dt*k3,       m, Iy, a, b, kf_axle, kr_axle, cf_axle, cr_axle, zr_f, zrdot_f, zr_r, zrdot_r);

    X(:, i+1) = X_i + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

fprintf('Done. %d steps.\n\n', N);

%% ---- Extract & Derive ------------------------------------------
z         = X(1, :);           % CG heave              [m]
zdot      = X(2, :);           % CG heave rate         [m/s]
theta     = X(3, :);           % Pitch angle           [rad]
thetaDot  = X(4, :);           % Pitch rate            [rad/s]
thetaDeg  = rad2deg(theta);

% Axle vertical displacements (sprung body at axle location)
z_front   = z - a .* theta;    % Front axle corner     [m]
z_rear    = z + b .* theta;    % Rear  axle corner     [m]

% Convert to mm for plotting
z_front_mm = z_front * 1e3;
z_rear_mm  = z_rear  * 1e3;
z_mm       = z       * 1e3;

% Road profile for overlay
road_f = arrayfun(zr_f, t_vec) * 1e3;
road_r = arrayfun(zr_r, t_vec) * 1e3;

%% ---- Metrics ---------------------------------------------------
fprintf('--- Axle Response Metrics ---\n');
fprintf('Peak Front Axle Displacement : %.4f mm\n', max(abs(z_front_mm)));
fprintf('Peak Rear  Axle Displacement : %.4f mm\n', max(abs(z_rear_mm)));
fprintf('Peak Pitch Angle             : %.4f deg\n', max(abs(thetaDeg)));
fprintf('Peak CG Heave                : %.4f mm\n', max(abs(z_mm)));

%% ============================================================
%  PLOTTING
%% ============================================================
col_front  = [0.00  0.82  1.00];   % Cyan    - front axle
col_rear   = [1.00  0.42  0.12];   % Orange  - rear  axle
col_cg     = [0.55  1.00  0.45];   % Green   - CG heave
col_pitch  = [0.90  0.30  0.90];   % Magenta - pitch
col_road   = [0.80  0.80  0.30];   % Yellow  - road input
col_grid   = [0.25  0.25  0.30];
col_txt    = [0.92  0.92  0.92];

fig = figure('Name', 'Pitching Ride - Bump Excitation', ...
             'Color', [0.05 0.05 0.08], ...
             'Position', [80 60 1350 900]);

%-----------------------------------------------------------------
% Subplot 1: Front & Rear Axle Displacement + Road Input
%-----------------------------------------------------------------
ax1 = subplot(3, 2, [1 2]);
hold on;

% Shade bump passage regions
fill_bump_patch(t_vec, t_bump_dur, 0,           ax1, [0.80 0.80 0.20]);   % Front bump
fill_bump_patch(t_vec, t_bump_dur, t_delay_r,   ax1, [1.00 0.50 0.10]);   % Rear  bump

% Road profiles (dashed)
plot(t_vec, road_f, '--', 'Color', [col_road 0.55], 'LineWidth', 1.2, 'DisplayName', 'Road (Front)');
plot(t_vec, road_r, '--', 'Color', [col_rear  0.55], 'LineWidth', 1.2, 'DisplayName', 'Road (Rear)');

% Axle responses
plot(t_vec, z_front_mm, 'Color', col_front, 'LineWidth', 2.0, 'DisplayName', 'Front Axle z_f');
plot(t_vec, z_rear_mm,  'Color', col_rear,  'LineWidth', 2.0, 'DisplayName', 'Rear  Axle z_r');
plot(t_vec, z_mm,       'Color', col_cg,    'LineWidth', 1.4, 'LineStyle', '-.', 'DisplayName', 'CG Heave z');

yline(0, '-', 'Color', [1 1 1 0.15], 'LineWidth', 0.8);

xlabel('Time (s)',             'Color', col_txt, 'FontSize', 11);
ylabel('Displacement (mm)',    'Color', col_txt, 'FontSize', 11);
title('Front & Rear Axle Displacement — Bump Excitation', ...
      'Color', col_txt, 'FontSize', 13, 'FontWeight', 'bold');
lg1 = legend('Location', 'northeast', 'TextColor', col_txt, ...
             'Color', [0.08 0.08 0.12], 'EdgeColor', col_grid, 'FontSize', 8);
applyDarkAxes(ax1, col_grid, col_txt);

%-----------------------------------------------------------------
% Subplot 2: Pitch Angle
%-----------------------------------------------------------------
ax2 = subplot(3, 2, [3 4]);
hold on;

fill_bump_patch(t_vec, t_bump_dur, 0,         ax2, [0.80 0.80 0.20]);
fill_bump_patch(t_vec, t_bump_dur, t_delay_r, ax2, [1.00 0.50 0.10]);

plot(t_vec, thetaDeg, 'Color', col_pitch, 'LineWidth', 2.0);
yline(0, '-', 'Color', [1 1 1 0.15], 'LineWidth', 0.8);

xlabel('Time (s)',          'Color', col_txt, 'FontSize', 11);
ylabel('Pitch \theta (deg)', 'Color', col_txt, 'FontSize', 11);
title('Pitch Angle Response', 'Color', col_txt, 'FontSize', 13, 'FontWeight', 'bold');
legend('Pitch \theta(t)', 'Location', 'northeast', ...
       'TextColor', col_txt, 'Color', [0.08 0.08 0.12], 'EdgeColor', col_grid);
applyDarkAxes(ax2, col_grid, col_txt);

%-----------------------------------------------------------------
% Subplot 3: Road Input Profile (both axles)
%-----------------------------------------------------------------
ax3 = subplot(3, 2, 5);
hold on;

area(t_vec, road_f, 'FaceColor', col_front, 'FaceAlpha', 0.20, ...
     'EdgeColor', col_front, 'LineWidth', 1.4, 'DisplayName', 'Front Road z_{r,f}');
area(t_vec, road_r, 'FaceColor', col_rear,  'FaceAlpha', 0.20, ...
     'EdgeColor', col_rear,  'LineWidth', 1.4, 'DisplayName', 'Rear  Road z_{r,r}');

% Annotate delay arrow
y_arr = H_bump*1e3 * 0.6;
annotation_x1 = t_delay_r / t_end;
text(t_bump_dur*0.5,     y_arr*1.25, 'Front', 'Color', col_front, ...
     'FontSize', 8, 'HorizontalAlignment', 'center');
text(t_delay_r + t_bump_dur*0.5, y_arr*1.25, 'Rear', 'Color', col_rear, ...
     'FontSize', 8, 'HorizontalAlignment', 'center');

xlabel('Time (s)',       'Color', col_txt, 'FontSize', 11);
ylabel('Road z_r (mm)', 'Color', col_txt, 'FontSize', 11);
title(sprintf('Bump Profile  (H = %.0f mm,  Delay = %.3f s)', H_bump*1e3, t_delay_r), ...
      'Color', col_txt, 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'TextColor', col_txt, ...
       'Color', [0.08 0.08 0.12], 'EdgeColor', col_grid, 'FontSize', 8);
applyDarkAxes(ax3, col_grid, col_txt);

%-----------------------------------------------------------------
% Subplot 4: Phase Portrait  z_front vs z_rear
%-----------------------------------------------------------------
ax4 = subplot(3, 2, 6);
hold on;

numSeg = 300;
idx_s  = round(linspace(1, N, numSeg+1));
cmap   = cool(numSeg);

% Plot the trajectory with a gradient color
for s = 1:numSeg
    seg = idx_s(s):idx_s(s+1);
    plot(z_front_mm(seg), z_rear_mm(seg), ...
         'Color', [cmap(s,:) 0.85], 'LineWidth', 1.4);
end

% Mark start point in green
plot(z_front_mm(1), z_rear_mm(1), 'o', 'MarkerSize', 8, ...
     'MarkerFaceColor', [0.2 1 0.4], 'MarkerEdgeColor', 'w', 'LineWidth', 1.2, ...
     'DisplayName', 'Start Point');

% Mark end point in red
plot(z_front_mm(end), z_rear_mm(end), 's', 'MarkerSize', 8, ...
     'MarkerFaceColor', [1 0.3 0.3], 'MarkerEdgeColor', 'w', 'LineWidth', 1.2, ...
     'DisplayName', 'End Point');

% Diagonal unity line
ax_lim = max(abs([z_front_mm z_rear_mm])) * 1.1;
plot([-ax_lim ax_lim], [-ax_lim ax_lim], '--', ...
     'Color', [1 1 1 0.15], 'LineWidth', 0.8, 'DisplayName', 'Unity Line');

xlabel('Front Axle Displacement (mm)', 'Color', col_txt, 'FontSize', 11);
ylabel('Rear  Axle Displacement (mm)', 'Color', col_txt, 'FontSize', 11);
title('Phase Portrait  z_{front} vs z_{rear}', 'Color', col_txt, 'FontSize', 13, 'FontWeight', 'bold');
legend('Trajectory', 'Start Point', 'End Point', ...
       'Location', 'northwest', 'TextColor', col_txt, ...
       'Color', [0.08 0.08 0.12], 'EdgeColor', col_grid, 'FontSize', 8);
applyDarkAxes(ax4, col_grid, col_txt);

%-----------------------------------------------------------------
% Super-title
%-----------------------------------------------------------------
annotation('textbox', [0 0.97 1 0.03], ...
    'String', sprintf( ...
      'Bump Ride (2-DOF)  |  V = %.1f m/s  |  H = %.0f mm  |  L_{bump} = %.2f m  |  Delay = %.3f s  |  omega_n(pitch) = %.3f rad/s  |  zeta = %.3f', ...
       V_vehicle, H_bump*1e3, L_bump, t_delay_r, omega_n, zeta), ...
    'Color', [0.65 0.65 0.65], 'FontSize', 9.5, ...
    'HorizontalAlignment', 'center', ...
    'EdgeColor', 'none', 'BackgroundColor', 'none');

set(fig, 'Units', 'normalized');
fprintf('Plot rendered successfully.\n');

%% ============================================================
%  LOCAL FUNCTIONS
%% ============================================================

function dX = heavePitchEOM(t, X, m, Iy, a, b, kf, kr, cf, cr, ...
                              zr_f, zrdot_f, zr_r, zrdot_r)
% -----------------------------------------------------------
%  2-DOF Heave + Pitch EOM
%  X = [z; zdot; theta; thetadot]
% -----------------------------------------------------------
    z        = X(1);
    zdot     = X(2);
    theta    = X(3);
    thetadot = X(4);

    % Suspension deflections (relative to road)
    def_f    = z - a*theta   - zr_f(t);
    defdot_f = zdot - a*thetadot - zrdot_f(t);
    def_r    = z + b*theta   - zr_r(t);
    defdot_r = zdot + b*thetadot - zrdot_r(t);

    % Spring + damper forces at each corner
    Ff = kf*def_f + cf*defdot_f;   % Front suspension force [N]
    Fr = kr*def_r + cr*defdot_r;   % Rear  suspension force [N]

    dX       = zeros(4, 1);
    dX(1)    = zdot;
    dX(2)    = -(Ff + Fr) / m;
    dX(3)    = thetadot;
    dX(4)    = (a*Ff - b*Fr) / Iy;
end

% -----------------------------------------------------------
function fill_bump_patch(t_vec, dur, t0, ax, fc)
% Shade the region where bump is active on a given axis
% -----------------------------------------------------------
    yl   = ylim(ax);
    ylo  = yl(1) - abs(yl(1))*0.5;   % extend below
    yhi  = yl(2) + abs(yl(2))*0.5;   % extend above
    t1   = t0;
    t2   = t0 + dur;
    fill(ax, [t1 t2 t2 t1], [ylo ylo yhi yhi], fc, ...
         'FaceAlpha', 0.10, 'EdgeColor', 'none');
end

% -----------------------------------------------------------
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

% Get all open figure handles
figHandles = findall(0, 'Type', 'figure');
