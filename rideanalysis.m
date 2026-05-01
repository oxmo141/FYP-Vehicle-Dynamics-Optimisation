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
m    = car.m_sprung;
Iy   = 212.09;
a    = car.a;
b    = car.b;
L    = car.wheelbase;

kf_axle = 2 * front.Kr;
kr_axle = 2 * rear.Kr;
cf_axle = 2 * front.cs;
cr_axle = 2 * rear.cs;

%% ---- Derived Properties ----------------------------------------
K_pitch = kf_axle*a^2 + kr_axle*b^2;
C_pitch = cf_axle*a^2 + cr_axle*b^2;
omega_n = sqrt(K_pitch / Iy);
zeta    = C_pitch / (2*sqrt(K_pitch * Iy));

omega_b = sqrt((kf_axle + kr_axle) / m);
zeta_b  = (cf_axle + cr_axle) / (2*sqrt((kf_axle+kr_axle)*m));

fprintf('--- System Properties ---\n');
fprintf('Pitch  : omega_n = %.4f rad/s  |  zeta = %.4f\n', omega_n, zeta);
fprintf('Bounce : omega_n = %.4f rad/s  |  zeta = %.4f\n', omega_b, zeta_b);
if zeta < 1
    fprintf('Pitch mode: UNDERDAMPED\n\n');
else
    fprintf('Pitch mode: OVERDAMPED\n\n');
end

%% ---- Bump Profile (Versine) ------------------------------------
V_vehicle  = 35.9 * 1000/3600;
H_bump     = 0.07;
L_bump     = 0.2;

t_bump_dur = L_bump / V_vehicle;
t_delay_r  = L / V_vehicle;

bump_disp = @(t, t0) (H_bump/2) * (1 - cos(2*pi*V_vehicle*(t-t0)/L_bump)) ...
                      .* (t >= t0) .* (t <= t0 + t_bump_dur);
bump_vel  = @(t, t0) (H_bump/2) * (2*pi*V_vehicle/L_bump) ...
                      .* sin(2*pi*V_vehicle*(t-t0)/L_bump) ...
                      .* (t >= t0) .* (t <= t0 + t_bump_dur);

zr_f    = @(t) bump_disp(t, 0);
zrdot_f = @(t) bump_vel(t,  0);
zr_r    = @(t) bump_disp(t, t_delay_r);
zrdot_r = @(t) bump_vel(t,  t_delay_r);

%% ---- Time Integration ------------------------------------------
X0    = [0; 0; 0; 0];
t_end = 0.5;
dt    = 0.001;
t_vec = 0 : dt : t_end;
N     = length(t_vec);
X     = zeros(4, N);
X(:,1)= X0;

fprintf('Running RK4 (2-DOF heave+pitch)...\n');
for i = 1 : N-1
    t_i = t_vec(i);  X_i = X(:,i);
    k1 = heavePitchEOM(t_i,      X_i,         m,Iy,a,b,kf_axle,kr_axle,cf_axle,cr_axle,zr_f,zrdot_f,zr_r,zrdot_r);
    k2 = heavePitchEOM(t_i+dt/2, X_i+dt/2*k1, m,Iy,a,b,kf_axle,kr_axle,cf_axle,cr_axle,zr_f,zrdot_f,zr_r,zrdot_r);
    k3 = heavePitchEOM(t_i+dt/2, X_i+dt/2*k2, m,Iy,a,b,kf_axle,kr_axle,cf_axle,cr_axle,zr_f,zrdot_f,zr_r,zrdot_r);
    k4 = heavePitchEOM(t_i+dt,   X_i+dt*k3,   m,Iy,a,b,kf_axle,kr_axle,cf_axle,cr_axle,zr_f,zrdot_f,zr_r,zrdot_r);
    X(:,i+1) = X_i + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end
fprintf('Done. %d steps.\n\n', N);

%% ---- Extract & Derive ------------------------------------------
z         = X(1,:);
thetaDeg  = rad2deg(X(3,:));
z_front_mm = (z - a.*X(3,:)) * 1e3;
z_rear_mm  = (z + b.*X(3,:)) * 1e3;
z_mm       = z * 1e3;
road_f     = arrayfun(zr_f, t_vec) * 1e3;
road_r     = arrayfun(zr_r, t_vec) * 1e3;

%% ---- Metrics ---------------------------------------------------
peak_front_mm  = max(abs(z_front_mm));
peak_rear_mm   = max(abs(z_rear_mm));
peak_pitch_deg = max(abs(thetaDeg));
peak_cg_mm     = max(abs(z_mm));

fprintf('--- Axle Response Metrics ---\n');
fprintf('Peak Front Axle Displacement : %.4f mm\n', peak_front_mm);
fprintf('Peak Rear  Axle Displacement : %.4f mm\n', peak_rear_mm);
fprintf('Peak Pitch Angle             : %.4f deg\n', peak_pitch_deg);
fprintf('Peak CG Heave                : %.4f mm\n', peak_cg_mm);

%% ============================================================
%  PLOTTING
%% ============================================================
col_front = [0.00  0.82  1.00];
col_rear  = [1.00  0.42  0.12];
col_cg    = [0.55  1.00  0.45];
col_pitch = [0.90  0.30  0.90];
col_road  = [0.80  0.80  0.30];
col_grid  = [0.25  0.25  0.30];
col_txt   = [0.92  0.92  0.92];

fig = figure('Name','Pitching Ride - Bump Excitation', ...
             'Color',[0.05 0.05 0.08], ...
             'Position',[80 60 1400 900]);

%% ---- Manual layout (all axes use 'Position' in norm. coords) ---
%
%  Horizontal zones:
%    lm=0.055   left margin
%    rm=0.015   right margin
%    c1=0.440   right edge of bump-profile axes
%    c2=0.470   left  edge of phase-portrait axes
%    c3=0.715   right edge of phase-portrait axes
%    c4=0.740   left  edge of summary panel  (annotation, no axes)
%
%  Vertical zones (bottom=0, top=1):
%    top row  : yb=0.680  yt=0.945
%    mid row  : yb=0.375  yt=0.622
%    bot row  : yb=0.055  yt=0.300
%
lm = 0.055;  rm = 0.015;
c1 = 0.440;  c2 = 0.470;  c3 = 0.715;  c4 = 0.740;

%-----------------------------------------------------------------
% Row 1: Axle displacement (full width)
%-----------------------------------------------------------------
ax1 = axes('Position',[lm  0.680  (1-lm-rm)  0.265]);
hold on;
fill_bump_patch(t_vec, t_bump_dur, 0,         ax1, [0.80 0.80 0.20]);
fill_bump_patch(t_vec, t_bump_dur, t_delay_r, ax1, [1.00 0.50 0.10]);
plot(t_vec, road_f, '--', 'Color',[col_road 0.55], 'LineWidth',1.2, 'DisplayName','Road (Front)');
plot(t_vec, road_r, '--', 'Color',[col_rear  0.55], 'LineWidth',1.2, 'DisplayName','Road (Rear)');
plot(t_vec, z_front_mm, 'Color',col_front, 'LineWidth',2.0, 'DisplayName','Front Axle z_f');
plot(t_vec, z_rear_mm,  'Color',col_rear,  'LineWidth',2.0, 'DisplayName','Rear  Axle z_r');
plot(t_vec, z_mm,       'Color',col_cg,    'LineWidth',1.4, 'LineStyle','-.', 'DisplayName','CG Heave z');
yline(0,'-','Color',[1 1 1 0.15],'LineWidth',0.8);
xlabel('Time (s)',          'Color',col_txt,'FontSize',11);
ylabel('Displacement (mm)', 'Color',col_txt,'FontSize',11);
title('Front & Rear Axle Displacement — Bump Excitation','Color',col_txt,'FontSize',13,'FontWeight','bold');
legend('Location','northeast','TextColor',col_txt,'Color',[0.08 0.08 0.12],'EdgeColor',col_grid,'FontSize',8);
applyDarkAxes(ax1, col_grid, col_txt);

%-----------------------------------------------------------------
% Row 2: Pitch angle (full width)
%-----------------------------------------------------------------
ax2 = axes('Position',[lm  0.375  (1-lm-rm)  0.247]);
hold on;
fill_bump_patch(t_vec, t_bump_dur, 0,         ax2, [0.80 0.80 0.20]);
fill_bump_patch(t_vec, t_bump_dur, t_delay_r, ax2, [1.00 0.50 0.10]);
plot(t_vec, thetaDeg, 'Color',col_pitch, 'LineWidth',2.0);
yline(0,'-','Color',[1 1 1 0.15],'LineWidth',0.8);
xlabel('Time (s)',           'Color',col_txt,'FontSize',11);
ylabel('Pitch \theta (deg)', 'Color',col_txt,'FontSize',11);
title('Pitch Angle Response','Color',col_txt,'FontSize',13,'FontWeight','bold');
legend('Pitch \theta(t)','Location','northeast', ...
       'TextColor',col_txt,'Color',[0.08 0.08 0.12],'EdgeColor',col_grid);
applyDarkAxes(ax2, col_grid, col_txt);

%-----------------------------------------------------------------
% Row 3 Col A: Bump profile
%-----------------------------------------------------------------
ax3 = axes('Position',[lm  0.055  (c1-lm)  0.245]);
hold on;
area(t_vec, road_f, 'FaceColor',col_front,'FaceAlpha',0.20,'EdgeColor',col_front,'LineWidth',1.4,'DisplayName','Front Road z_{r,f}');
area(t_vec, road_r, 'FaceColor',col_rear, 'FaceAlpha',0.20,'EdgeColor',col_rear, 'LineWidth',1.4,'DisplayName','Rear  Road z_{r,r}');
y_arr = H_bump*1e3*0.6;
text(t_bump_dur*0.5,             y_arr*1.25,'Front','Color',col_front,'FontSize',8,'HorizontalAlignment','center');
text(t_delay_r+t_bump_dur*0.5,  y_arr*1.25,'Rear', 'Color',col_rear, 'FontSize',8,'HorizontalAlignment','center');
xlabel('Time (s)',       'Color',col_txt,'FontSize',11);
ylabel('Road z_r (mm)', 'Color',col_txt,'FontSize',11);
title(sprintf('Bump Profile  (H = %.0f mm,  Delay = %.3f s)', H_bump*1e3, t_delay_r), ...
      'Color',col_txt,'FontSize',12,'FontWeight','bold');
legend('Location','northeast','TextColor',col_txt,'Color',[0.08 0.08 0.12],'EdgeColor',col_grid,'FontSize',8);
applyDarkAxes(ax3, col_grid, col_txt);

%-----------------------------------------------------------------
% Row 3 Col B: Phase portrait
%-----------------------------------------------------------------
ax4 = axes('Position',[c2  0.055  (c3-c2)  0.245]);
hold on;
numSeg = 300;
idx_s  = round(linspace(1,N,numSeg+1));
cmap   = cool(numSeg);
for s = 1:numSeg
    seg = idx_s(s):idx_s(s+1);
    plot(z_front_mm(seg), z_rear_mm(seg),'Color',[cmap(s,:) 0.85],'LineWidth',1.4);
end
plot(z_front_mm(1),  z_rear_mm(1),  'o','MarkerSize',8,'MarkerFaceColor',[0.2 1 0.4],'MarkerEdgeColor','w','LineWidth',1.2,'DisplayName','Start Point');
plot(z_front_mm(end),z_rear_mm(end),'s','MarkerSize',8,'MarkerFaceColor',[1 0.3 0.3],'MarkerEdgeColor','w','LineWidth',1.2,'DisplayName','End Point');
ax_lim = max(abs([z_front_mm z_rear_mm]))*1.1;
plot([-ax_lim ax_lim],[-ax_lim ax_lim],'--','Color',[1 1 1 0.15],'LineWidth',0.8,'DisplayName','Unity Line');
xlabel('Front Axle Disp. (mm)','Color',col_txt,'FontSize',10);
ylabel('Rear  Axle Disp. (mm)','Color',col_txt,'FontSize',10);
title('Phase Portrait  z_{front} vs z_{rear}','Color',col_txt,'FontSize',12,'FontWeight','bold');
legend('Trajectory','Start Point','End Point','Location','northwest', ...
       'TextColor',col_txt,'Color',[0.08 0.08 0.12],'EdgeColor',col_grid,'FontSize',8);
applyDarkAxes(ax4, col_grid, col_txt);

%-----------------------------------------------------------------
% Super-title
%-----------------------------------------------------------------
annotation('textbox',[0 0.97 1 0.03], ...
    'String', sprintf('Bump Ride (2-DOF)  |  V = %.1f m/s  |  H = %.0f mm  |  L_bump = %.2f m  |  Delay = %.3f s  |  wn(pitch) = %.4f rad/s  |  zeta = %.4f', ...
                      V_vehicle, H_bump*1e3, L_bump, t_delay_r, omega_n, zeta), ...
    'Color',[0.65 0.65 0.65],'FontSize',9.5, ...
    'HorizontalAlignment','center','EdgeColor','none','BackgroundColor','none');

%-----------------------------------------------------------------
% Row 3 Col C: Simulation summary panel
%   Occupies the horizontal gap [c4 .. 1-rm], same height as bot row.
%   Pure annotation — no axes object — so it can never be covered
%   by subplot auto-resize events.
%-----------------------------------------------------------------
px = c4;
py = 0.055;
pw = 1 - rm - c4;
ph = 0.245;

stats_str = { ...
    '--- MODAL PROPERTIES ---', ...
    sprintf('Pitch  wn  =  %.4f rad/s', omega_n), ...
    sprintf('Pitch  z   =  %.4f',       zeta), ...
    sprintf('Bounce wn  =  %.4f rad/s', omega_b), ...
    sprintf('Bounce z   =  %.4f',       zeta_b), ...
    '', ...
    '--- PEAK RESPONSES ---', ...
    sprintf('Front axle  =  %.4f mm',  peak_front_mm), ...
    sprintf('Rear  axle  =  %.4f mm',  peak_rear_mm), ...
    sprintf('Pitch angle =  %.4f deg', peak_pitch_deg), ...
    sprintf('CG heave    =  %.4f mm',  peak_cg_mm) };

% Background card
annotation('rectangle',[px py pw ph], ...
    'Color',[0.35 0.35 0.42],'FaceColor',[0.07 0.07 0.11],'LineWidth',0.8);

% Header
hdr_h = 0.036;
annotation('textbox',[px, py+ph-hdr_h, pw, hdr_h], ...
    'String','  SIMULATION SUMMARY', ...
    'Color',[0.85 0.85 0.95],'FontSize',8.5,'FontWeight','bold', ...
    'HorizontalAlignment','left','VerticalAlignment','middle', ...
    'EdgeColor','none','BackgroundColor','none');

% Divider
annotation('line',[px+0.005, px+pw-0.005],[py+ph-hdr_h, py+ph-hdr_h], ...
    'Color',[0.40 0.40 0.52],'LineWidth',0.7);

% Content
annotation('textbox',[px+0.007, py+0.008, pw-0.010, ph-hdr_h-0.010], ...
    'String',stats_str, ...
    'Color',[0.78 0.78 0.85],'FontSize',8.0,'FontName','Courier New', ...
    'HorizontalAlignment','left','VerticalAlignment','top', ...
    'EdgeColor','none','BackgroundColor','none');

set(fig,'Units','normalized');
fprintf('Plot rendered successfully.\n');

%% ============================================================
%  LOCAL FUNCTIONS
%% ============================================================

function dX = heavePitchEOM(t, X, m, Iy, a, b, kf, kr, cf, cr, ...
                              zr_f, zrdot_f, zr_r, zrdot_r)
    z = X(1); zdot = X(2); theta = X(3); thetadot = X(4);
    def_f    = z - a*theta    - zr_f(t);
    defdot_f = zdot - a*thetadot - zrdot_f(t);
    def_r    = z + b*theta    - zr_r(t);
    defdot_r = zdot + b*thetadot - zrdot_r(t);
    Ff = kf*def_f + cf*defdot_f;
    Fr = kr*def_r + cr*defdot_r;
    dX    = zeros(4,1);
    dX(1) = zdot;
    dX(2) = -(Ff+Fr)/m;
    dX(3) = thetadot;
    dX(4) = (a*Ff - b*Fr)/Iy;
end

function fill_bump_patch(t_vec, dur, t0, ax, fc)
    yl  = ylim(ax);
    ylo = yl(1) - abs(yl(1))*0.5;
    yhi = yl(2) + abs(yl(2))*0.5;
    fill(ax,[t0 t0+dur t0+dur t0],[ylo ylo yhi yhi],fc,'FaceAlpha',0.10,'EdgeColor','none');
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
    grid(ax,'on');
    hold(ax,'off');
end

figHandles = findall(0,'Type','figure');