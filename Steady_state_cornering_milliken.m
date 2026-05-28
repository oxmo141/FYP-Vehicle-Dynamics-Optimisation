%% Steady-State Cornering — Milliken Bicycle Model
%  Vehicle parameters  : paramR26.m  (FSAE car struct)
%  Tyre model          : MF61.m      (Magic Formula 6.1)
%  Tyre data           : 43105_18x7_5_10_R25B_7.tir
%
%  Cornering stiffness C_F and C_R are extracted by numerically
%  differentiating Fy w.r.t. alpha at alpha=0 for each axle load,
%  using the MF6.1 pure-slip path (kappa = 0).
%
%  Stability derivatives and control response ratios follow
%  Milliken & Milliken Ch.5, Eqs. (5.9)–(5.17).

clear; clc; close all;

%% =====================================================================
%  1. LOAD VEHICLE PARAMETERS FROM paramR26
%% =====================================================================
run('paramR26.m');          % populates: car, front, rear structs + g

m   = car.m;                % Total vehicle mass            [kg]
Iz  = YAW_INERTIA;          % Yaw moment of inertia         [kg·m²]
a   = car.a;                % CG to front axle              [m]
b   = car.b;                % CG to rear axle               [m]
L   = car.wheelbase;        % Wheelbase                     [m]
Fz0 = 1080;                 % Nominal tyre load from TIR    [N]

fprintf('=========================================================\n');
fprintf('  MILLIKEN BICYCLE MODEL — FSAE R26 / MF6.1 TYRE\n');
fprintf('=========================================================\n');
fprintf('\n--- Vehicle Parameters (from paramR26) ---\n');
fprintf('  Total mass          m   = %.2f kg\n',  m);
fprintf('  Yaw inertia         Iz  = %.2f kg·m²\n', Iz);
fprintf('  CG to front axle    a   = %.4f m\n',  a);
fprintf('  CG to rear  axle    b   = %.4f m\n',  b);
fprintf('  Wheelbase           L   = %.4f m\n',  L);
fprintf('  Weight distribution     = %.1f%% front\n', (a/L)*100);

%% =====================================================================
%  2. LOAD TIR FILE — parse key=value pairs into a struct
%% =====================================================================
tirFile = '43105_18x7.5_10_R25B_7.tir';
tirParams = parseTIR(tirFile);

% Fill in range defaults if not present in TIR
tirParams = setTIRdefaults(tirParams);

fprintf('\n--- Tyre File Loaded: %s ---\n', tirFile);
fprintf('  Nominal load  Fz0     = %.0f N\n',  tirParams.FNOMIN);
fprintf('  Nominal press pi0     = %.0f Pa\n', tirParams.NOMPRES);
fprintf('  Nominal speed V0      = %.1f m/s\n',tirParams.LONGVL);

%% =====================================================================
%  3. COMPUTE AXLE LOADS (static, level ground)
%% =====================================================================
%  Front axle load = m*g * (b/L),  rear = m*g * (a/L)
Fzf_static = m * g * (b / L);    % Total front axle load [N]
Fzr_static = m * g * (a / L);    % Total rear  axle load [N]
Fzf_corner = Fzf_static / 2;     % Per-tyre front        [N]
Fzr_corner = Fzr_static / 2;     % Per-tyre rear         [N]

fprintf('\n--- Static Axle Loads ---\n');
fprintf('  Front axle total  = %.1f N  (%.1f N / tyre)\n', Fzf_static, Fzf_corner);
fprintf('  Rear  axle total  = %.1f N  (%.1f N / tyre)\n', Fzr_static, Fzr_corner);

%% =====================================================================
%  4. EXTRACT CORNERING STIFFNESS FROM MF6.1
%
%  Cornering stiffness C = dFy/d(alpha) |_{alpha=0}
%  Computed by central finite difference on the MF6.1 pure-slip output.
%  kappa = 0 (pure lateral), gamma = 0, nominal pressure, nominal speed.
%
%  C_F and C_R are the AXLE stiffnesses (2 x per-tyre value).
%% =====================================================================
pnom  = tirParams.NOMPRES;
Vcx   = tirParams.LONGVL;    % Use nominal tyre speed for linearisation
dalpha = 1e-4;               % Finite difference step  [rad]

% Per-tyre cornering stiffness via central difference
Cf_tyre = cornStiff_MF61(Fzf_corner, dalpha, pnom, Vcx, tirParams);
Cr_tyre = cornStiff_MF61(Fzr_corner, dalpha, pnom, Vcx, tirParams);

% Axle stiffness = 2 tyres per axle
CF = 2 * Cf_tyre;    % Front axle cornering stiffness  [N/rad]
CR = 2 * Cr_tyre;    % Rear  axle cornering stiffness  [N/rad]

fprintf('\n--- MF6.1 Cornering Stiffness  [dFy/dalpha at alpha=0] ---\n');
fprintf('  Front per-tyre  Cf  = %8.1f N/rad  (Fz = %.1f N)\n', Cf_tyre, Fzf_corner);
fprintf('  Rear  per-tyre  Cr  = %8.1f N/rad  (Fz = %.1f N)\n', Cr_tyre, Fzr_corner);
fprintf('  Front axle      C_F = %8.1f N/rad  (2 x per-tyre)\n', CF);
fprintf('  Rear  axle      C_R = %8.1f N/rad  (2 x per-tyre)\n', CR);

% Also show how Kya varies with load (useful diagnostic)
Fz_sweep = linspace(100, 3*Fz0, 40);
Kya_sweep = arrayfun(@(fz) cornStiff_MF61(fz, dalpha, pnom, Vcx, tirParams), Fz_sweep);

%% =====================================================================
%  5. STABILITY & CONTROL DERIVATIVES  — Eq. (5.9)
%% =====================================================================
Y_beta  =  CF + CR;
Y_delta = -CF;
N_beta  =  a*CF - b*CR;
N_delta = -a*CF;

Y_r = @(V)  (1/V) * (a*CF - b*CR);
N_r = @(V) -(1/V) * (a^2*CF + b^2*CR);

fprintf('\n--- Stability & Control Derivatives  [Eq. 5.9] ---\n');
fprintf('  Y_beta  = C_F + C_R                = %+10.1f  N/rad\n',       Y_beta);
fprintf('  Y_delta = -C_F                      = %+10.1f  N/rad\n',       Y_delta);
fprintf('  N_beta  = a*C_F - b*C_R             = %+10.1f  N·m/rad\n',    N_beta);
fprintf('  N_delta = -a*C_F                    = %+10.1f  N·m/rad\n',    N_delta);
fprintf('  Y_r(V)  = (1/V)(a*C_F - b*C_R)    [speed-dependent]\n');
fprintf('  N_r(V)  = -(1/V)(a²C_F + b²C_R)   [speed-dependent]\n');

V_ex = 15;   % representative FSAE corner speed [m/s] ~54 km/h
fprintf('\n  At V = %.0f m/s:\n', V_ex);
fprintf('    Y_r = %+10.2f  N/(rad/s)\n',   Y_r(V_ex));
fprintf('    N_r = %+10.2f  N·m/(rad/s)\n', N_r(V_ex));

%% =====================================================================
%  6. UNDERSTEER GRADIENT  — tyre-stiffness form
%% =====================================================================
K_us       = (m / L^2) * (b/CF - a/CR);          % [rad·s²/m]
K_us_deg_g = K_us * g * (180/pi);                 % [deg/g]

fprintf('\n--- Understeer Gradient ---\n');
fprintf('  K_us = (m/L²)(b/C_F - a/C_R)\n');
fprintf('       = %.6f  rad·s²/m   (%.4f  deg/g)\n', K_us, K_us_deg_g);
if K_us > 0
    V_char = sqrt(L*g / K_us);
    fprintf('  → UNDERSTEER  |  V_char = %.2f m/s  (%.1f km/h)\n', V_char, V_char*3.6);
elseif K_us < 0
    V_crit_us = sqrt(-L*g / K_us);
    fprintf('  → OVERSTEER   |  V_crit = %.2f m/s  (%.1f km/h)\n', V_crit_us, V_crit_us*3.6);
else
    fprintf('  → NEUTRAL STEER\n');
end

%% =====================================================================
%  7. STABILITY DETERMINANT  Q(V)  — Eq. (5.14)
%     Q = N_beta*Y_r - N_beta*m*V - Y_beta*N_r
%% =====================================================================
Q_fun = @(V)  N_beta*Y_r(V) - N_beta*m*V - Y_beta*N_r(V);

fprintf('\n--- Stability Determinant  Q  [Eq. 5.14] ---\n');
fprintf('  Q(V) = N_beta*Y_r(V) - N_beta*m*V - Y_beta*N_r(V)\n\n');

V_test = [5, 10, 15, 20, 25, 30];
fprintf('  %-12s  %-14s\n', 'Speed [m/s]', 'Q  [N²·s/m]');
fprintf('  %s\n', repmat('-',1,28));
for V_i = V_test
    fprintf('  %-12.0f  %+14.1f\n', V_i, Q_fun(V_i));
end

% Find if Q = 0 exists in operating range
U_fine      = 1:0.05:80;
Q_fine      = arrayfun(Q_fun, U_fine);
sc          = find(diff(sign(Q_fine)));
if ~isempty(sc)
    V_Qzero = U_fine(sc(1));
    fprintf('\n  Q = 0 at V ≈ %.2f m/s  (%.1f km/h) — oversteer divergence speed\n', ...
        V_Qzero, V_Qzero*3.6);
else
    fprintf('\n  Q > 0 throughout operating range → stable understeer response\n');
end

%% =====================================================================
%  8. CONTROL RESPONSE RATIOS  — Eqs. (5.13)–(5.17)
%
%  Common numerator (speed-independent):
%    NUM_curv = Y_beta*N_delta - N_beta*Y_delta
%
%  (1/R)/delta  = NUM_curv / (V*Q)                      Eq.(5.13)
%  r/delta      = NUM_curv / Q                           Eq.(5.15)
%  (V²/R)/delta = V*NUM_curv / Q                        Eq.(5.16)
%  beta/delta   = (Y_delta*N_r - N_delta*(Y_r-mV)) / Q  Eq.(5.17)
%% =====================================================================
NUM_curv = Y_beta*N_delta - N_beta*Y_delta;

fprintf('\n--- Numerator  Y_beta*N_delta - N_beta*Y_delta ---\n');
fprintf('  = (%.1f)(%.1f) - (%.1f)(%.1f) = %.2f  N²·m/rad²\n', ...
    Y_beta, N_delta, N_beta, Y_delta, NUM_curv);

fprintf('\n--- Steady-State Control Responses  (delta = 5 deg) ---\n');
delta_deg = 5;
delta_rad = delta_deg * pi/180;
fprintf('  %-10s  %-10s  %-10s  %-14s  %-12s  %-12s\n', ...
    'V [m/s]','V [km/h]','1/R [1/m]','R [m]','r [deg/s]','ay [g]','beta [deg]');
fprintf('  %-10s  %-10s  %-10s  %-14s  %-12s  %-12s\n', ...
    '','','','','','');
fprintf('  %s\n', repmat('-',1,78));

for V_i = V_test
    Q_i  = Q_fun(V_i);
    NB_i = Y_delta*N_r(V_i) - N_delta*(Y_r(V_i) - m*V_i);
    cr   = NUM_curv / (V_i * Q_i);             % (1/R)/delta [1/m per rad]
    yr   = NUM_curv / Q_i;                      % r/delta
    la   = V_i * NUM_curv / Q_i;               % (V²/R)/delta
    br   = NB_i / Q_i;                         % beta/delta

    fprintf('  %-10.0f  %-10.1f  %-10.5f  %-14.1f  %-12.2f  %-12.4f  %.3f\n', ...
        V_i, V_i*3.6, cr*delta_rad, 1/(cr*delta_rad), ...
        yr*delta_rad*(180/pi), la*delta_rad/g, br*delta_rad*(180/pi));
end

%% =====================================================================
%  9. PLOTS
%% =====================================================================
U_vec = linspace(3, 35, 400);     % FSAE relevant speed range [m/s]
U_kph = U_vec * 3.6;

Q_vec      = arrayfun(Q_fun, U_vec);
curv_vec   = NUM_curv ./ (U_vec .* Q_vec);
yaw_vec    = NUM_curv ./ Q_vec;
latA_vec   = U_vec .* NUM_curv ./ Q_vec;
NB_vec     = arrayfun(@(V) Y_delta*N_r(V) - N_delta*(Y_r(V) - m*V), U_vec);
beta_vec   = NB_vec ./ Q_vec;

% --- Common title font size for all subplots ---
tfont = 11;   % subplot title font size
lfont = 10;   % axis label font size

fig = figure('Name','FSAE R26 — Milliken Steady-State Cornering', ...
             'NumberTitle','off', 'Position',[40 40 1500 820]);

% (a) Cornering stiffness vs vertical load
ax1 = subplot(2,4,1);
plot(Fz_sweep, Kya_sweep/1000, 'k-', 'LineWidth', 2); hold on;
xline(Fzf_corner, 'b--', 'LineWidth', 1.2);
xline(Fzr_corner, 'r--', 'LineWidth', 1.2);
xline(Fz0,        'g:',  'LineWidth', 1.2);
hold off;
xlabel('Vertical load  F_z  [N]',        'FontSize', lfont);
ylabel('Cornering stiffness  [kN/rad]',  'FontSize', lfont);
title('Cornering Stiffness vs Load', 'FontSize', tfont);
legend('K_{ya}(F_z)', 'F_{zf}', 'F_{zr}', 'F_{z,nom}', ...
       'Location','southeast', 'FontSize', 9);
grid on; box on;

% (b) Stability determinant Q
ax2 = subplot(2,4,2);
plot(U_kph, Q_vec/1e6, 'k-', 'LineWidth', 2);
yline(0, 'r--', 'LineWidth', 1.2);
xlabel('Speed  [km/h]',                 'FontSize', lfont);
ylabel('Q  [\times10^6 N^2 \cdot s/m]', 'FontSize', lfont);
title('Stability Determinant  Q',       'FontSize', tfont);
grid on; box on;

% (c) Curvature response (1/R)/delta
ax3 = subplot(2,4,3);
plot(U_kph, curv_vec*(180/pi), 'b-', 'LineWidth', 2);
xlabel('Speed  [km/h]',                       'FontSize', lfont);
ylabel('(1/R)/\delta  [m^{-1} rad^{-1}]',     'FontSize', lfont);
title('Curvature Response',                   'FontSize', tfont);
grid on; box on;

% (d) Yaw rate response r/delta
ax4 = subplot(2,4,4);
plot(U_kph, yaw_vec*(180/pi), 'r-', 'LineWidth', 2);
xlabel('Speed  [km/h]',                       'FontSize', lfont);
ylabel('r/\delta  [deg s^{-1} deg^{-1}]',     'FontSize', lfont);
title('Yaw Rate Response',                    'FontSize', tfont);
grid on; box on;
if K_us > 0
    xline(V_char*3.6, 'm--', 'V_{char}', ...
          'LabelHorizontalAlignment','left', 'FontSize', 9);
end

% (e) Lateral acceleration response (V²/R)/delta
ax5 = subplot(2,4,5);
plot(U_kph, latA_vec/g*(180/pi), 'Color',[0.1 0.6 0.1], 'LineWidth', 2);
xlabel('Speed  [km/h]',                       'FontSize', lfont);
ylabel('(V^2/R)/\delta  [g rad^{-1}]',        'FontSize', lfont);
title('Lateral Acceleration Response',        'FontSize', tfont);
grid on; box on;

% (f) Sideslip angle response beta/delta
ax6 = subplot(2,4,6);
plot(U_kph, beta_vec*(180/pi), 'm-', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Speed  [km/h]',                       'FontSize', lfont);
ylabel('\beta/\delta  [deg deg^{-1}]',        'FontSize', lfont);
title('Body Sideslip Response',               'FontSize', tfont);
grid on; box on;

% (g) Speed-dependent stability derivatives Y_r and N_r
ax7 = subplot(2,4,7);
Yr_vec = arrayfun(Y_r, U_vec);
Nr_vec = arrayfun(N_r, U_vec);
yyaxis left
plot(U_kph, Yr_vec/1000, 'b-', 'LineWidth', 2);
ylabel('Y_r  [kN \cdot s rad^{-1}]', 'FontSize', lfont);
yyaxis right
plot(U_kph, Nr_vec/1000, 'r--', 'LineWidth', 2);
ylabel('N_r  [kN \cdot m \cdot s rad^{-1}]', 'FontSize', lfont);
xlabel('Speed  [km/h]', 'FontSize', lfont);
title('Damping Derivatives  Y_r  and  N_r',  'FontSize', tfont);
legend('Y_r', 'N_r', 'Location','northeast', 'FontSize', 9);
grid on; box on;

% (h) Fy vs slip angle at front and rear operating loads
ax8 = subplot(2,4,8);
alpha_sweep = linspace(-0.3, 0.3, 200);
Fy_f = arrayfun(@(al) MF61_pureLat(Fzf_corner, al, pnom, Vcx, tirParams), alpha_sweep);
Fy_r = arrayfun(@(al) MF61_pureLat(Fzr_corner, al, pnom, Vcx, tirParams), alpha_sweep);
plot(alpha_sweep*(180/pi), Fy_f/1000, 'b-', 'LineWidth', 2); hold on;
plot(alpha_sweep*(180/pi), Fy_r/1000, 'r-', 'LineWidth', 2);
hold off;
xlabel('Slip angle  \alpha  [deg]',           'FontSize', lfont);
ylabel('Lateral force  F_y  [kN]',            'FontSize', lfont);
title('Lateral Force vs Slip Angle',          'FontSize', tfont);
legend(sprintf('Front  F_z = %.0f N', Fzf_corner), ...
       sprintf('Rear   F_z = %.0f N', Fzr_corner), ...
       'Location','northwest', 'FontSize', 9);
grid on; box on;

% Super-title — kept short so it fits without clipping
sgtitle(sprintf('FSAE R26  —  Steady-State Cornering  |  C_F = %.0f N/rad,   C_R = %.0f N/rad', ...
        CF, CR), 'FontSize', 12, 'FontWeight', 'bold');

% Apply consistent tight layout spacing
set([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8], 'TitleFontSizeMultiplier', 1, ...
    'LabelFontSizeMultiplier', 1);
fig.PaperPositionMode = 'auto';

fprintf('\n=== Analysis Complete ===\n');

%% =====================================================================
%  LOCAL FUNCTIONS
%% =====================================================================

function C = cornStiff_MF61(Fz, da, pres, Vcx, tp)
    % Cornering stiffness = dFy/dalpha at alpha=0, kappa=0, gamma=0
    % Central finite difference
    kappa = 0;  gamma = 0;
    [~, Fyp] = MF61(Fz, kappa,  da, gamma, Vcx, pres, tp);
    [~, Fym] = MF61(Fz, kappa, -da, gamma, Vcx, pres, tp);
    C = (Fyp - Fym) / (2*da);
end

function Fy = MF61_pureLat(Fz, alpha, pres, Vcx, tp)
    % Convenience wrapper: pure lateral Fy
    [~, Fy] = MF61(Fz, 0, alpha, 0, Vcx, pres, tp);
end

function tirParams = parseTIR(filename)
    % Parse a .tir file into a struct.
    % Reads every  KEY = VALUE  line; ignores section headers and comments.
    tirParams = struct();
    fid = fopen(filename, 'r');
    if fid < 0
        error('Cannot open TIR file: %s', filename);
    end
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line) || line(1)=='$' || line(1)=='['; continue; end
        tok = regexp(line, '^\s*(\w+)\s*=\s*([^\$]+)', 'tokens', 'once');
        if isempty(tok); continue; end
        key = strtrim(tok{1});
        val = strtrim(tok{2});
        numVal = str2double(val);
        if ~isnan(numVal)
            tirParams.(key) = numVal;
        else
            tirParams.(key) = strrep(strrep(val, '''', ''), '"', '');
        end
    end
    fclose(fid);
end

function tp = setTIRdefaults(tp)
    % Supply sensible default limits if not present in the TIR file.
    % Cell array is Nx2: {fieldName, defaultValue} — one row per field.
    defaults = { ...
        'FZMIN',    50;      ...
        'FZMAX',    4000;    ...
        'KPUMIN',  -0.5;     ...
        'KPUMAX',   0.5;     ...
        'ALPMIN',  -0.5;     ...
        'ALPMAX',   0.5;     ...
        'CAMMIN',  -0.175;   ...
        'CAMMAX',   0.175;   ...
        'PRESMIN',  60000;   ...
        'PRESMAX',  120000;  ...
        'VXLOW',    1        };

    for k = 1:size(defaults, 1)      % iterate rows, not columns
        field = defaults{k, 1};
        if ~isfield(tp, field)
            tp.(field) = defaults{k, 2};
        end
    end

    % Ensure all scaling factors default to 1 if missing from TIR
    scaleFields = {'LFZO','LCX','LMUX','LEX','LKX','LHX','LVX', ...
                   'LCY','LMUY','LEY','LKY','LHY','LVY', ...
                   'LKYC','LKZC','LXAL','LVMX','LMX','LMY','LMP'};
    for k = 1:numel(scaleFields)
        if ~isfield(tp, scaleFields{k})
            tp.(scaleFields{k}) = 1;
        end
    end
end