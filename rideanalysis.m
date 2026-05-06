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

%% ---- Overdamping Delay Analysis --------------------------------
%
%  For a 2nd-order system: s^2 + 2*zeta*wn*s + wn^2 = 0
%
%  Overdamped (zeta > 1) real poles:
%    s1 = -wn*(zeta - sqrt(zeta^2 - 1))   [slow pole]
%    s2 = -wn*(zeta + sqrt(zeta^2 - 1))   [fast pole]
%
%  Time constants:  tau = -1/s
%    tau_slow = 1 / (wn*(zeta - sqrt(zeta^2-1)))
%    tau_fast = 1 / (wn*(zeta + sqrt(zeta^2-1)))
%
%  Critical reference (zeta = 1):
%    tau_crit = 1/wn   (repeated pole at -wn)
%
%  Overdamping delay = additional time for the SLOW mode to decay
%  relative to critical damping:
%    delay = tau_slow - tau_crit
%
%  Peak-response delay (step input): time to reach 63.2% of static
%  deflection is tau_slow for the overdamped case vs tau_crit for
%  critical — the difference is the "slow-mode excess" delay.
%
%  For underdamped modes, delay < 0 (system responds faster but
%  overshoots). We report the magnitude and sign.
%
%  Settling time (2% band) for 2nd-order:
%    Overdamped : t_settle ~ 4 * tau_slow  (slow mode dominates)
%    Critical   : t_settle ~ 4 / wn
%    delta_settle = 4*(tau_slow - tau_crit)  = 4*delay

% ---- Pitch mode -------------------------------------------------
if zeta >= 1
    aux_p        = sqrt(zeta^2 - 1);
    s1_p         = -omega_n * (zeta - aux_p);   % slow pole (less negative)
    s2_p         = -omega_n * (zeta + aux_p);   % fast pole
    tau_slow_p   = -1 / s1_p;
    tau_fast_p   = -1 / s2_p;
else
    % Underdamped: complex poles, use envelope time constant
    tau_slow_p   = 1 / (zeta * omega_n);
    tau_fast_p   = tau_slow_p;                  % same envelope
end
tau_crit_p   = 1 / omega_n;                    % reference: zeta=1
delay_p      = tau_slow_p - tau_crit_p;        % +ve = slower than critical
settle_p     = 4 * tau_slow_p;
settle_crit_p= 4 * tau_crit_p;
delta_settle_p = settle_p - settle_crit_p;

% ---- Bounce mode ------------------------------------------------
if zeta_b >= 1
    aux_b        = sqrt(zeta_b^2 - 1);
    s1_b         = -omega_b * (zeta_b - aux_b);
    s2_b         = -omega_b * (zeta_b + aux_b);
    tau_slow_b   = -1 / s1_b;
    tau_fast_b   = -1 / s2_b;
else
    tau_slow_b   = 1 / (zeta_b * omega_b);
    tau_fast_b   = tau_slow_b;
end
tau_crit_b   = 1 / omega_b;
delay_b      = tau_slow_b - tau_crit_b;
settle_b     = 4 * tau_slow_b;
settle_crit_b= 4 * tau_crit_b;
delta_settle_b = settle_b - settle_crit_b;

fprintf('--- Overdamping Delay Analysis (ref: zeta = 1) ---\n');
fprintf('\n  PITCH MODE\n');
fprintf('  tau_crit  (zeta=1)   = %.4f s\n', tau_crit_p);
fprintf('  tau_slow  (actual)   = %.4f s\n', tau_slow_p);
fprintf('  tau_fast  (actual)   = %.4f s\n', tau_fast_p);
fprintf('  Delay vs critical    = %+.4f s  (%s)\n', delay_p, ...
        ternary(delay_p >= 0, 'SLOWER — overdamped', 'FASTER — underdamped'));
fprintf('  Settle (actual)      = %.4f s\n', settle_p);
fprintf('  Settle (zeta=1 ref)  = %.4f s\n', settle_crit_p);
fprintf('  Delta settle         = %+.4f s\n', delta_settle_p);

fprintf('\n  BOUNCE MODE\n');
fprintf('  tau_crit  (zeta=1)   = %.4f s\n', tau_crit_b);
fprintf('  tau_slow  (actual)   = %.4f s\n', tau_slow_b);
fprintf('  tau_fast  (actual)   = %.4f s\n', tau_fast_b);
fprintf('  Delay vs critical    = %+.4f s  (%s)\n', delay_b, ...
        ternary(delay_b >= 0, 'SLOWER — overdamped', 'FASTER — underdamped'));
fprintf('  Settle (actual)      = %.4f s\n', settle_b);
fprintf('  Settle (zeta=1 ref)  = %.4f s\n', settle_crit_b);
fprintf('  Delta settle         = %+.4f s\n\n', delta_settle_b);

%% ---- Step Response Curves (for overdamping delay plot) ---------
%  Unit-step response of a 2nd-order system, normalised to static = 1.
%
%  Overdamped (zeta > 1):
%    y(t) = 1 - e^(s1*t)*(s2/(s2-s1)) + e^(s2*t)*(s1/(s2-s1))   [incorrect sign form]
%    Cleaner: use MATLAB's step() via tf or direct formula:
%
%    y(t) = 1 + A1*exp(s1*t) + A2*exp(s2*t)
%    where A1 = -s2/(s2-s1),  A2 = s1/(s2-s1)   (unit-step, DC gain=1)
%
%  Critical (zeta = 1):
%    y(t) = 1 - (1 + wn*t)*exp(-wn*t)
%
%  Underdamped (zeta < 1):
%    wd = wn*sqrt(1-zeta^2)
%    y(t) = 1 - exp(-zeta*wn*t)*(cos(wd*t) + (zeta/sqrt(1-zeta^2))*sin(wd*t))

t_step = linspace(0, max(settle_p, settle_b) * 1.6, 2000);

% Pitch — actual
y_pitch_actual = stepResponse2DOF(t_step, omega_n, zeta);
% Pitch — critical reference
y_pitch_crit   = stepResponse2DOF(t_step, omega_n, 1.0);

% Bounce — actual
y_bounce_actual = stepResponse2DOF(t_step, omega_b, zeta_b);
% Bounce — critical reference
y_bounce_crit   = stepResponse2DOF(t_step, omega_b, 1.0);

% 63.2% crossing times (approximate tau via interpolation)
tau63_pitch_actual  = interp1(y_pitch_actual,  t_step, 0.632, 'linear', NaN);
tau63_pitch_crit    = interp1(y_pitch_crit,    t_step, 0.632, 'linear', NaN);
tau63_bounce_actual = interp1(y_bounce_actual, t_step, 0.632, 'linear', NaN);
tau63_bounce_crit   = interp1(y_bounce_crit,   t_step, 0.632, 'linear', NaN);

fprintf('--- 63.2%% Crossing Times ---\n');
fprintf('  Pitch  actual  : %.4f s\n', tau63_pitch_actual);
fprintf('  Pitch  crit    : %.4f s\n', tau63_pitch_crit);
fprintf('  Bounce actual  : %.4f s\n', tau63_bounce_actual);
fprintf('  Bounce crit    : %.4f s\n\n', tau63_bounce_crit);

%% ---- Bump Profile (Versine) ------------------------------------
V_vehicle  = 60 * 1000/3600;
H_bump     = 0.0254;
L_bump     = 0.05;

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
z          = X(1,:);
thetaDeg   = rad2deg(X(3,:));
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
%  FIGURE 1 — Ride Analysis (original layout)
%% ============================================================
col_front = [0.00  0.82  1.00];
col_rear  = [1.00  0.42  0.12];
col_cg    = [0.55  1.00  0.45];
col_pitch = [0.90  0.30  0.90];
col_road  = [0.80  0.80  0.30];
col_grid  = [0.25  0.25  0.30];
col_txt   = [0.92  0.92  0.92];

fig1 = figure('Name','Pitching Ride - Bump Excitation', ...
              'Color',[0.05 0.05 0.08], ...
              'Position',[80 60 1400 900]);

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
text(t_bump_dur*0.5,            y_arr*1.25,'Front','Color',col_front,'FontSize',8,'HorizontalAlignment','center');
text(t_delay_r+t_bump_dur*0.5, y_arr*1.25,'Rear', 'Color',col_rear, 'FontSize',8,'HorizontalAlignment','center');
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
cmap_pp = cool(numSeg);
for s = 1:numSeg
    seg = idx_s(s):idx_s(s+1);
    plot(z_front_mm(seg), z_rear_mm(seg),'Color',[cmap_pp(s,:) 0.85],'LineWidth',1.4);
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
% Super-title (Fig 1)
%-----------------------------------------------------------------
annotation('textbox',[0 0.97 1 0.03], ...
    'String', sprintf('Bump Ride (2-DOF)  |  V = %.1f m/s  |  H = %.0f mm  |  L_bump = %.2f m  |  Delay = %.3f s  |  wn(pitch) = %.4f rad/s  |  zeta = %.4f', ...
                      V_vehicle, H_bump*1e3, L_bump, t_delay_r, omega_n, zeta), ...
    'Color',[0.65 0.65 0.65],'FontSize',9.5, ...
    'HorizontalAlignment','center','EdgeColor','none','BackgroundColor','none');

%-----------------------------------------------------------------
% Row 3 Col C: Simulation summary panel (original)
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

annotation('rectangle',[px py pw ph], ...
    'Color',[0.35 0.35 0.42],'FaceColor',[0.07 0.07 0.11],'LineWidth',0.8);

hdr_h = 0.036;
annotation('textbox',[px, py+ph-hdr_h, pw, hdr_h], ...
    'String','  SIMULATION SUMMARY', ...
    'Color',[0.85 0.85 0.95],'FontSize',8.5,'FontWeight','bold', ...
    'HorizontalAlignment','left','VerticalAlignment','middle', ...
    'EdgeColor','none','BackgroundColor','none');

annotation('line',[px+0.005, px+pw-0.005],[py+ph-hdr_h, py+ph-hdr_h], ...
    'Color',[0.40 0.40 0.52],'LineWidth',0.7);

annotation('textbox',[px+0.007, py+0.008, pw-0.010, ph-hdr_h-0.010], ...
    'String',stats_str, ...
    'Color',[0.78 0.78 0.85],'FontSize',8.0,'FontName','Courier New', ...
    'HorizontalAlignment','left','VerticalAlignment','top', ...
    'EdgeColor','none','BackgroundColor','none');

set(fig1,'Units','normalized');
fprintf('Figure 1 rendered successfully.\n');

%% ============================================================
%  FIGURE 2 — Overdamping Delay Analysis
%% ============================================================
col_crit = [1.00  0.85  0.20];   % zeta=1 reference (yellow)
col_ap   = [0.30  0.80  1.00];   % actual pitch (cyan)
col_ab   = [0.55  1.00  0.45];   % actual bounce (green)
col_brk  = [1.00  0.50  0.20];   % bracket / delta annotation (orange)
bg_lbl   = [0.08  0.08  0.12];   % label background box colour

fig2 = figure('Name','Overdamping Delay Analysis', ...
              'Color',[0.05 0.05 0.08], ...
              'Position',[120 100 1300 860]);

lm2      = 0.070;  rm2 = 0.025;
col_gap2 = 0.055;
col_w2   = (1 - lm2 - rm2 - col_gap2) / 2;

% ---- Helper: draw a bracket with end-ticks and centred label ----
%   drawBracket(ax, x1, x2, y, tickH, label, col, fs)
%   All lines use HandleVisibility='off' so they never enter the legend.

%-----------------------------------------------------------------
% Panel A (left): PITCH — step response comparison
%-----------------------------------------------------------------
axA = axes('Position',[lm2  0.100  col_w2  0.810]);
hold on;

mask_p = t_step <= settle_p * 1.5;
tp  = t_step(mask_p);           tp  = tp(:)';
ypa = y_pitch_actual(mask_p);   ypa = ypa(:)';
ypc = y_pitch_crit(mask_p);     ypc = ypc(:)';

% Shaded region between curves
patch([tp, fliplr(tp)], [ypa, fliplr(ypc)], col_crit, ...
      'FaceAlpha', 0.13, 'EdgeColor', 'none', 'HandleVisibility', 'off');

% Main curves (legend entries)
plot(tp, ypc, '--', 'Color', col_crit, 'LineWidth', 2.0, ...
     'DisplayName', sprintf('\\zeta = 1   (\\tau_{crit} = %.4f s)', tau_crit_p));
plot(tp, ypa, '-',  'Color', col_ap,   'LineWidth', 2.5, ...
     'DisplayName', sprintf('\\zeta = %.4f  (\\tau_{slow} = %.4f s)', zeta, tau_slow_p));

% 63.2% horizontal — no label, no legend
yline(0.632, ':', 'Color', [col_txt 0.35], 'LineWidth', 1.0, ...
      'HandleVisibility', 'off');
% Unity horizontal
yline(1.0, '-', 'Color', [col_txt 0.12], 'LineWidth', 0.8, ...
      'HandleVisibility', 'off');

% 63.2% text label at left edge (clear space, no overlap)
text(tp(1), 0.632, '  63.2%', 'Color', col_txt, 'FontSize', 9, ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');

% Vertical marker lines — no labels, no legend
if ~isnan(tau63_pitch_crit)
    xline(tau63_pitch_crit,  '--', 'Color', [col_crit 0.70], ...
          'LineWidth', 1.2, 'HandleVisibility', 'off');
end
if ~isnan(tau63_pitch_actual)
    xline(tau63_pitch_actual,'--', 'Color', [col_ap   0.70], ...
          'LineWidth', 1.2, 'HandleVisibility', 'off');
end
xline(settle_crit_p, ':', 'Color', [col_crit 0.45], ...
      'LineWidth', 1.0, 'HandleVisibility', 'off');
xline(settle_p,      ':', 'Color', [col_ap   0.45], ...
      'LineWidth', 1.0, 'HandleVisibility', 'off');

% tau labels — placed BELOW x-axis as text (outside axes clip, using
% 'Units','data' and clipping off) — instead put them just above x=0
% at the bottom margin of the plot, with background boxes.
if ~isnan(tau63_pitch_crit)
    text(tau63_pitch_crit, -0.035, sprintf('\\tau_{crit}\n%.4f s', tau63_pitch_crit), ...
         'Color', col_crit, 'FontSize', 8.5, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
         'BackgroundColor', bg_lbl, 'Margin', 1);
end
if ~isnan(tau63_pitch_actual)
    text(tau63_pitch_actual, -0.035, sprintf('\\tau_{slow}\n%.4f s', tau63_pitch_actual), ...
         'Color', col_ap, 'FontSize', 8.5, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
         'BackgroundColor', bg_lbl, 'Margin', 1);
end
text(settle_crit_p, -0.090, sprintf('t_{s,crit}\n%.4f s', settle_crit_p), ...
     'Color', col_crit, 'FontSize', 8, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
     'BackgroundColor', bg_lbl, 'Margin', 1);
text(settle_p, -0.090, sprintf('t_{s,act}\n%.4f s', settle_p), ...
     'Color', col_ap, 'FontSize', 8, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
     'BackgroundColor', bg_lbl, 'Margin', 1);

% Bracket: Δτ₆₃ at 63.2% level
if ~isnan(tau63_pitch_crit) && ~isnan(tau63_pitch_actual)
    drawBracket(axA, tau63_pitch_crit, tau63_pitch_actual, 0.50, 0.022, ...
        sprintf('\\Delta\\tau_{63} = %.4f s', tau63_pitch_actual - tau63_pitch_crit), ...
        col_brk, 10);
end

% Bracket: Δt_settle near top
drawBracket(axA, settle_crit_p, settle_p, 1.10, 0.022, ...
    sprintf('\\Delta t_{settle} = %+.4f s', delta_settle_p), ...
    col_brk, 10);

xlabel('Time (s)',            'Color', col_txt, 'FontSize', 12);
ylabel('Normalised response', 'Color', col_txt, 'FontSize', 12);
title(sprintf('PITCH MODE — Overdamping Delay  (\\omega_n = %.4f rad/s)', omega_n), ...
      'Color', col_txt, 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'southeast', 'TextColor', col_txt, ...
       'Color', [0.08 0.08 0.12], 'EdgeColor', col_grid, 'FontSize', 10);
ylim([-0.16 1.20]);
applyDarkAxes(axA, col_grid, col_txt);

%-----------------------------------------------------------------
% Panel B (right): BOUNCE — step response comparison
%-----------------------------------------------------------------
axB = axes('Position',[lm2+col_w2+col_gap2  0.100  col_w2  0.810]);
hold on;

mask_b = t_step <= settle_b * 1.5;
tb  = t_step(mask_b);            tb  = tb(:)';
yba = y_bounce_actual(mask_b);   yba = yba(:)';
ybc = y_bounce_crit(mask_b);     ybc = ybc(:)';

patch([tb, fliplr(tb)], [yba, fliplr(ybc)], col_crit, ...
      'FaceAlpha', 0.13, 'EdgeColor', 'none', 'HandleVisibility', 'off');

plot(tb, ybc, '--', 'Color', col_crit, 'LineWidth', 2.0, ...
     'DisplayName', sprintf('\\zeta = 1   (\\tau_{crit} = %.4f s)', tau_crit_b));
plot(tb, yba, '-',  'Color', col_ab,   'LineWidth', 2.5, ...
     'DisplayName', sprintf('\\zeta = %.4f  (\\tau_{slow} = %.4f s)', zeta_b, tau_slow_b));

yline(0.632, ':', 'Color', [col_txt 0.35], 'LineWidth', 1.0, ...
      'HandleVisibility', 'off');
yline(1.0,   '-', 'Color', [col_txt 0.12], 'LineWidth', 0.8, ...
      'HandleVisibility', 'off');

text(tb(1), 0.632, '  63.2%', 'Color', col_txt, 'FontSize', 9, ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');

if ~isnan(tau63_bounce_crit)
    xline(tau63_bounce_crit,  '--', 'Color', [col_crit 0.70], ...
          'LineWidth', 1.2, 'HandleVisibility', 'off');
end
if ~isnan(tau63_bounce_actual)
    xline(tau63_bounce_actual,'--', 'Color', [col_ab   0.70], ...
          'LineWidth', 1.2, 'HandleVisibility', 'off');
end
xline(settle_crit_b, ':', 'Color', [col_crit 0.45], ...
      'LineWidth', 1.0, 'HandleVisibility', 'off');
xline(settle_b,      ':', 'Color', [col_ab   0.45], ...
      'LineWidth', 1.0, 'HandleVisibility', 'off');

if ~isnan(tau63_bounce_crit)
    text(tau63_bounce_crit, -0.035, sprintf('\\tau_{crit}\n%.4f s', tau63_bounce_crit), ...
         'Color', col_crit, 'FontSize', 8.5, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
         'BackgroundColor', bg_lbl, 'Margin', 1);
end
if ~isnan(tau63_bounce_actual)
    text(tau63_bounce_actual, -0.035, sprintf('\\tau_{slow}\n%.4f s', tau63_bounce_actual), ...
         'Color', col_ab, 'FontSize', 8.5, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
         'BackgroundColor', bg_lbl, 'Margin', 1);
end
text(settle_crit_b, -0.090, sprintf('t_{s,crit}\n%.4f s', settle_crit_b), ...
     'Color', col_crit, 'FontSize', 8, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
     'BackgroundColor', bg_lbl, 'Margin', 1);
text(settle_b, -0.090, sprintf('t_{s,act}\n%.4f s', settle_b), ...
     'Color', col_ab, 'FontSize', 8, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
     'BackgroundColor', bg_lbl, 'Margin', 1);

if ~isnan(tau63_bounce_crit) && ~isnan(tau63_bounce_actual)
    drawBracket(axB, tau63_bounce_crit, tau63_bounce_actual, 0.50, 0.022, ...
        sprintf('\\Delta\\tau_{63} = %.4f s', tau63_bounce_actual - tau63_bounce_crit), ...
        col_brk, 10);
end

drawBracket(axB, settle_crit_b, settle_b, 1.10, 0.022, ...
    sprintf('\\Delta t_{settle} = %+.4f s', delta_settle_b), ...
    col_brk, 10);

xlabel('Time (s)',            'Color', col_txt, 'FontSize', 12);
ylabel('Normalised response', 'Color', col_txt, 'FontSize', 12);
title(sprintf('BOUNCE MODE — Overdamping Delay  (\\omega_n = %.4f rad/s)', omega_b), ...
      'Color', col_txt, 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'southeast', 'TextColor', col_txt, ...
       'Color', [0.08 0.08 0.12], 'EdgeColor', col_grid, 'FontSize', 10);
ylim([-0.16 1.20]);
applyDarkAxes(axB, col_grid, col_txt);

%-----------------------------------------------------------------
% Super-title (Fig 2)
%-----------------------------------------------------------------
annotation('textbox',[0 0.960 1 0.035], ...
    'String', sprintf(['Overdamping Delay  (ref: \\zeta = 1)  |  ' ...
    'Pitch: \\zeta = %.4f  delay = %+.4f s  \\Deltasettle = %+.4f s  |  ' ...
    'Bounce: \\zeta = %.4f  delay = %+.4f s  \\Deltasettle = %+.4f s'], ...
    zeta,   delay_p, delta_settle_p, ...
    zeta_b, delay_b, delta_settle_b), ...
    'Color',[0.65 0.65 0.65],'FontSize',9.5, ...
    'HorizontalAlignment','center','EdgeColor','none','BackgroundColor','none');

set(fig2,'Units','normalized');
fprintf('Figure 2 rendered successfully.\n');

%% ============================================================
%  LOCAL FUNCTIONS
%% ============================================================

function y = stepResponse2DOF(t, wn, zeta)
% STEPRESPONSE2DOF  Normalised unit-step response of a 2nd-order system.
%   y(t->inf) = 1  (static deflection normalised to 1)
%
%   Overdamped  (zeta > 1):
%     s1 = -wn*(zeta - sqrt(zeta^2-1))
%     s2 = -wn*(zeta + sqrt(zeta^2-1))
%     y  = 1 + (s2/(s1-s2))*exp(s1*t) - (s1/(s1-s2))*exp(s2*t)   [wrong]
%
%   Correct partial-fraction expansion for unit step with DC gain = wn^2:
%     Y(s) = wn^2 / [s*(s-s1)*(s-s2)]
%     y(t) = 1 + [s1/(s2*(s1/s2-1))]*exp(s2*t) - ...  <- use the formula below
%
%   Simplest form (verified):
%     y(t) = 1 - exp(s1*t)*(s2/(s2-s1)) + exp(s2*t)*(s1/(s2-s1))
%          = 1 + exp(s1*t)*s2/(s1-s2) + exp(s2*t)*s1/(s2-s1)
%
%   Critical (zeta = 1):
%     y(t) = 1 - (1 + wn*t)*exp(-wn*t)
%
%   Underdamped (zeta < 1):
%     wd = wn*sqrt(1-zeta^2)
%     y(t) = 1 - exp(-zeta*wn*t)*(cos(wd*t) + (zeta/sqrt(1-zeta^2))*sin(wd*t))

    if zeta > 1 + 1e-6
        aux = sqrt(zeta^2 - 1);
        s1  = -wn * (zeta - aux);   % slow (less negative)
        s2  = -wn * (zeta + aux);   % fast (more negative)
        % Partial fraction: A1 at s1, A2 at s2
        %   y(t) = 1 + A1*exp(s1*t) + A2*exp(s2*t)
        %   A1 = wn^2/(s1*(s1-s2))  [residue at s1 of wn^2/(s*(s-s1)*(s-s2))]
        %   A2 = wn^2/(s2*(s2-s1))
        A1  =  wn^2 / (s1 * (s1 - s2));
        A2  =  wn^2 / (s2 * (s2 - s1));
        y   = 1 + A1 .* exp(s1 .* t) + A2 .* exp(s2 .* t);

    elseif abs(zeta - 1) <= 1e-6
        % Critically damped
        y = 1 - (1 + wn .* t) .* exp(-wn .* t);

    else
        % Underdamped
        wd = wn * sqrt(1 - zeta^2);
        y  = 1 - exp(-zeta .* wn .* t) .* ...
             (cos(wd .* t) + (zeta / sqrt(1 - zeta^2)) .* sin(wd .* t));
    end
end

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
    fill(ax,[t0 t0+dur t0+dur t0],[ylo ylo yhi yhi], ...
         fc,'FaceAlpha',0.10,'EdgeColor','none');
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

function out = ternary(cond, a, b)
    if cond;  out = a;  else;  out = b;  end
end

function drawBracket(ax, x1, x2, y, tickH, label, col, fs)
% DRAWBRACKET  Draw a horizontal bracket with end-ticks and centred label.
%   All graphics elements use HandleVisibility='off' — never in legend.
    plot(ax, [x1 x2],   [y y],       '-', 'Color', col, 'LineWidth', 1.8, 'HandleVisibility', 'off');
    plot(ax, [x1 x1],   [y-tickH y+tickH], '-', 'Color', col, 'LineWidth', 1.8, 'HandleVisibility', 'off');
    plot(ax, [x2 x2],   [y-tickH y+tickH], '-', 'Color', col, 'LineWidth', 1.8, 'HandleVisibility', 'off');
    text(ax, mean([x1 x2]), y + tickH*1.8, label, ...
         'Color', col, 'FontSize', fs, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
         'BackgroundColor', [0.08 0.08 0.12], 'Margin', 2);
end

figHandles = findall(0,'Type','figure');