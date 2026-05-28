%% Yaw Moment Diagram (YMD) - Bicycle Model with MF6.1 Tires
%
% Speed sweep: 20, 40, 60 km/h — aerodynamic loads included.
% Cornering CL/CD/aero-balance used for YMD (lateral load case).
%
% Metrics follow DrRacing (2015) and Milliken & Milliken RCVD Ch.5 & Ch.8.
%
% ISO sign conventions (consistent with MF61.m):
%   beta  > 0  => nose-left     delta > 0 => steer left
%   Y     > 0  => force left    N     > 0 => yaw moment nose-left

clear; clc; close all;

%% ── 1. TIRE PARAMETERS ──────────────────────────────────────────────────
tirFile = '43105_18x7.5_10_R25B_7.tir';
tirParams = parseTIR(tirFile);

tirParams.FZMIN   = 10;      tirParams.FZMAX   = 5000;
tirParams.KPUMIN  = -1.5;    tirParams.KPUMAX  =  1.5;
tirParams.ALPMIN  = -0.6981; tirParams.ALPMAX   =  0.6981;
tirParams.CAMMIN  = -0.2618; tirParams.CAMMAX   =  0.2618;
tirParams.PRESMIN =  60000;  tirParams.PRESMAX  = 120000;

%% ── 2. VEHICLE PARAMETERS (loaded from R26params.m) ────────────────────
run('paramR26.m');   % populates: car, front, rear, frontsprung, rearsprung, etc.

g = 9.81;

% -- Mass
m = car.front_m + car.rear_m;              % total vehicle mass [kg]
weight_distribution = car.front_m / m;     % front mass fraction [-]

% -- Geometry
% YMD convention: a = CG to front axle, b = CG to rear axle.
% car.b in R26params = wheelbase * weight_distribution = CG-to-front distance.
% car.a in R26params = wheelbase - car.b                = CG-to-rear  distance.
a = car.b;            % CG to front axle [m]  (R26params car.b = front distance)
b = car.a;            % CG to rear  axle [m]  (R26params car.a = rear  distance)
L = car.wheelbase;    % wheelbase [m]

% -- Tyre / operating
pres  = 84000;        % tyre inflation pressure [Pa]
gamma = 0;            % camber [rad]
kappa = 0;            % slip ratio (pure lateral, no drive/brake)

fprintf('Vehicle parameters loaded from R26params.m:\n');
fprintf('  Total mass     : %.1f kg\n',  m);
fprintf('  Weight dist.   : %.1f%% front\n', weight_distribution*100);
fprintf('  a (CG->front)  : %.3f m\n',  a);
fprintf('  b (CG->rear)   : %.3f m\n',  b);
fprintf('  Wheelbase      : %.3f m\n',  L);

%% ── 2b. LATERAL LOAD TRANSFER PARAMETERS ───────────────────────────────
% From Milliken & Milliken RCVD — lateral load transfer per axle:
%   DW_F/Ay = (W/t_F) * [ H*K_F/(K_F+K_R)  +  (b/L)*z_RF ]
%   DW_R/Ay = (W/t_R) * [ H*K_R/(K_F+K_R)  +  (a/L)*z_RR ]
% where H = CG height - effective roll centre height [m]
%
% All roll stiffness values drawn from paramR26.m (front.k_roll, rear.k_roll).

t_F  = car.track;                  % front track width [m] (assumed equal tracks)
t_R  = car.track;                  % rear  track width [m]
W    = m * g;                      % total weight [N]

K_F  = front.k_roll;               % front roll stiffness [Nm/rad]
K_R  = rear.k_roll;                % rear  roll stiffness [Nm/rad]

z_RF = front.RC;                   % front roll centre height, n [m]
z_RR = rear.RC;                    % rear  roll centre height, m [m]
% Effective roll centre height from eq.1 (Fig.19): x = h - (a*n + b*m)/(a+b)
% where n = z_RF (front RC), m = z_RR (rear RC), h = CG height
% H = x = height of CG above the roll centreline
H    = car.cgh - (a*z_RF + b*z_RR) / L;  % [m]  (L = a+b = wheelbase)

% Pre-compute the Ay-normalised LLT gradients [N per g]
% (these are constant — geometry/stiffness only, not speed-dependent)
LLT_F_per_g = (W / t_F) * ( H * K_F/(K_F + K_R)  +  (b/L) * z_RF );
LLT_R_per_g = (W / t_R) * ( H * K_R/(K_F + K_R)  +  (a/L) * z_RR );

fprintf('Lateral load transfer gradients:\n');
fprintf('  K_F = %.1f Nm/rad | K_R = %.1f Nm/rad\n', K_F, K_R);
fprintf('  H (CG above eff. RC) = %.4f m\n', H);
fprintf('  LLT_F = %.1f N/g | LLT_R = %.1f N/g\n', LLT_F_per_g, LLT_R_per_g);

%% ── 3. AERODYNAMIC PARAMETERS ───────────────────────────────────────────
den   = 1.196;      % air density [kg/m^3]
farea = 1.157757;   % frontal area [m^2]
CLc   = 3.713;      % lift  coefficient (cornering)
CDc   = 1.412;      % drag  coefficient (cornering)
ab_c  = 0.528;      % aero balance cornering (fraction of downforce at front)

% Dynamic pressure helper: q(V) = 0.5 * den * farea * V^2
% Downforce (positive = pushes down):  Df = CLc * q
% Front downforce: Df_F = ab_c  * Df
% Rear  downforce: Df_R = (1-ab_c) * Df
% Note: drag acts longitudinally and is ignored in the lateral YMD.

%% ── 4. STEERING RATIO POLYNOMIAL ────────────────────────────────────────
% Average SR polynomial (bicycle model fit, R^2 = 0.999828):
%   delta_wheel [deg] = polyval(p_avg, SW)
% SW = steering wheel angle [deg], range +/-120 deg (240 deg lock-to-lock).
%
% Coefficients: p(1)*SW^4 + p(2)*SW^3 + p(3)*SW^2 + p(4)*SW + p(5)

% SR polynomial coefficients — polyval(p_avg, SW) gives the STEERING RATIO
% at that steering wheel angle. Wheel angle = SW / SR(SW).
p_avg = [-4.854730e-10, ...   % SW^4
         -7.950252e-22, ...   % SW^3
         -1.753110e-05, ...   % SW^2
         +4.930866e-18, ...   % SW^1
         +4.856089e+00];      % SW^0 — SR at centre (~4.856:1)

SW_max    = 120;                              % max steering wheel angle [deg]
nD        = 17;                              % number of delta lines on YMD
SW_sweep  = linspace(-SW_max, SW_max, nD);  % steering wheel angles [deg]

SR_sweep  = polyval(p_avg, SW_sweep);        % steering ratio at each SW [-]
delta_deg = SW_sweep ./ SR_sweep;            % wheel angle [deg] = SW / SR
delta     = deg2rad(delta_deg);              % [rad]

fprintf('Steering polynomial mapping (SR -> wheel angle):\n');
fprintf('  SW range    : +/-%.0f deg (lock-to-lock: %.0f deg)\n', SW_max, 2*SW_max);
fprintf('  SR range    : %.3f to %.3f\n', min(SR_sweep), max(SR_sweep));
fprintf('  delta range : %.3f to %.3f deg (wheel)\n', min(delta_deg), max(delta_deg));

%% ── 5. SWEEP DEFINITIONS ────────────────────────────────────────────────
V_kph     = [20, 40, 60];              % speed sweep [km/h]
V_sweep   = V_kph / 3.6;              % [m/s]
nV        = numel(V_sweep);

beta_deg  = linspace(-12, 12, 241);   % body slip angle [deg]
beta      = deg2rad(beta_deg);
nB        = numel(beta);

%% ── 5. SPEED LOOP ───────────────────────────────────────────────────────
for iV = 1:nV

    V = V_sweep(iV);

    % ── Aerodynamic loads ──────────────────────────────────────────────
    q    = 0.5 * den * farea * V^2;   % aero force scale [N]
    Df   = CLc * q;                    % total downforce [N]
    Df_F = ab_c       * Df;            % front downforce [N]
    Df_R = (1 - ab_c) * Df;           % rear  downforce [N]

    % Total axle vertical loads = static + aero downforce
    Fz_F = m * g * b / L + Df_F;
    Fz_R = m * g * a / L + Df_R;

    % ── Pre-allocate ───────────────────────────────────────────────────
    Y_mat   = zeros(nB, nD);
    N_mat   = zeros(nB, nD);
    FyF_mat = zeros(nB, nD);
    FyR_mat = zeros(nB, nD);
    alpF_mat= zeros(nB, nD);
    alpR_mat= zeros(nB, nD);
    dWF_mat = zeros(nB, nD);   % front LLT [N]
    dWR_mat = zeros(nB, nD);   % rear  LLT [N]

    % ── Sweep β and δ  (fixed-point iteration for LLT) ────────────────
    % LLT depends on Ay, which depends on tyre forces, which depend on Fz.
    % We iterate: guess Ay=0, compute forces -> Ay -> new LLT -> repeat.
    max_iter = 8;   tol = 1e-4;   % converges in 3-4 iterations typically

    for iD = 1:nD
        for iB = 1:nB
            alpha_F_ISO = -(beta(iB) + delta(iD));
            alpha_R_ISO = -(beta(iB));

            Ay_est = 0;   % initial guess [g]
            for iter = 1:max_iter
                % LLT at current Ay estimate
                dW_F = LLT_F_per_g * Ay_est;   % +ve = outer tyre loaded [N]
                dW_R = LLT_R_per_g * Ay_est;

                % Total axle loads including aero and LLT
                % Outer tyre gains load; inner tyre loses it.
                % Bicycle model uses single equivalent tyre per axle, so
                % we use the mean axle load (LLT cancels in the average).
                % Effect of LLT enters via the nonlinear Fy(Fz) — use the
                % outer/inner split and average the lateral forces:
                Fz_Fo = Fz_F + dW_F;   % outer front [N]
                Fz_Fi = Fz_F - dW_F;   % inner front [N]
                Fz_Ro = Fz_R + dW_R;   % outer rear  [N]
                Fz_Ri = Fz_R - dW_R;   % inner rear  [N]

                % Clamp to physically valid loads
                Fz_Fo = max(Fz_Fo, 0);  Fz_Fi = max(Fz_Fi, 0);
                Fz_Ro = max(Fz_Ro, 0);  Fz_Ri = max(Fz_Ri, 0);

                % MF6.1 for each tyre, then average per axle
                [~, Fy_Fo] = MF61(Fz_Fo, kappa, alpha_F_ISO, gamma, V, pres, tirParams);
                [~, Fy_Fi] = MF61(Fz_Fi, kappa, alpha_F_ISO, gamma, V, pres, tirParams);
                [~, Fy_Ro] = MF61(Fz_Ro, kappa, alpha_R_ISO, gamma, V, pres, tirParams);
                [~, Fy_Ri] = MF61(Fz_Ri, kappa, alpha_R_ISO, gamma, V, pres, tirParams);

                Fy_F_ax = (Fy_Fo + Fy_Fi) / 2;   % axle-average front [N]
                Fy_R_ax = (Fy_Ro + Fy_Ri) / 2;   % axle-average rear  [N]

                Ay_new = (Fy_F_ax + Fy_R_ax) / (m * g);   % [g]

                if abs(Ay_new - Ay_est) < tol; break; end
                Ay_est = Ay_new;
            end

            Y_mat(iB,iD)    = Fy_F_ax + Fy_R_ax;
            N_mat(iB,iD)    = Fy_F_ax * a - Fy_R_ax * b;
            FyF_mat(iB,iD)  = Fy_F_ax;
            FyR_mat(iB,iD)  = Fy_R_ax;
            alpF_mat(iB,iD) = alpha_F_ISO;
            alpR_mat(iB,iD) = alpha_R_ISO;
            dWF_mat(iB,iD)  = dW_F;
            dWR_mat(iB,iD)  = dW_R;
        end
    end

    Ay_mat = Y_mat / (m * g);   % lateral accel [g]

    % ── Extract metrics ────────────────────────────────────────────────
    M = extractMetrics(Ay_mat, N_mat, FyF_mat, FyR_mat, alpF_mat, alpR_mat, ...
                        beta_deg, delta_deg, SW_sweep, nB, nD, m, g);

    % ── Console table ─────────────────────────────────────────────────
    printMetricsTable(M, V, m, a, b, Fz_F, Fz_R, Df, Df_F, Df_R);

    % ── Figure ────────────────────────────────────────────────────────
    plot_YMD(V, V_kph(iV), m, a, b, Df, Df_F, Df_R, ...
            Ay_mat, N_mat, beta_deg, delta_deg, SW_sweep, nB, nD, M);

end % speed loop

%% ════════════════════════════════════════════════════════════════════════
%% FUNCTIONS
%% ════════════════════════════════════════════════════════════════════════

%% ── extractMetrics ──────────────────────────────────────────────────────
function M = extractMetrics(Ay_mat, N_mat, FyF_mat, FyR_mat, alpF_mat, alpR_mat, ...
                              beta_deg, delta_deg, SW_sweep, nB, nD, m, g)

    % 1 – Max Ay
    [maxAy, idx] = max(Ay_mat(:));
    [iB_mAy, iD_mAy] = ind2sub([nB nD], idx);
    M.MaxAy = maxAy;

    % 2 – Max Ay trimmed (N~0)
    trimAy = zeros(1,nD);
    for iD = 1:nD
        [~, ib] = min(abs(N_mat(:,iD)));
        trimAy(iD) = Ay_mat(ib,iD);
    end
    M.MaxAy_trimmed = max(trimAy);

    % 3 – N at Max Ay
    M.N_at_MaxAy = N_mat(iB_mAy, iD_mAy);

    % 4 – State at Max Ay
    M.alphaF_at_MaxAy = rad2deg(alpF_mat(iB_mAy, iD_mAy));
    M.alphaR_at_MaxAy = rad2deg(alpR_mat(iB_mAy, iD_mAy));
    M.beta_at_MaxAy   = beta_deg(iB_mAy);
    M.delta_at_MaxAy  = delta_deg(iD_mAy);
    M.SW_at_MaxAy     = SW_sweep(iD_mAy);    % steering wheel angle at max Ay [deg]
    M.FyF_at_MaxAy    = FyF_mat(iB_mAy, iD_mAy);
    M.FyR_at_MaxAy    = FyR_mat(iB_mAy, iD_mAy);

    % 5 – Max |N|
    [~, idx2] = max(abs(N_mat(:)));
    [iB_mN, iD_mN] = ind2sub([nB nD], idx2);
    M.MaxN = N_mat(iB_mN, iD_mN);

    % 6 – State at Max N
    M.alphaF_at_MaxN = rad2deg(alpF_mat(iB_mN, iD_mN));
    M.alphaR_at_MaxN = rad2deg(alpR_mat(iB_mN, iD_mN));
    M.beta_at_MaxN   = beta_deg(iB_mN);
    M.delta_at_MaxN  = delta_deg(iD_mN);
    M.Ay_at_MaxN     = Ay_mat(iB_mN, iD_mN);

    % 7 – dN/dd @ b=0
    [~, iB0] = min(abs(beta_deg));
    M.dN_dDelta_beta0 = mean(gradient(N_mat(iB0,:), delta_deg));

    % 8 – dN/dd @ b=bAymax
    M.dN_dDelta_betaAymax = mean(gradient(N_mat(iB_mAy,:), delta_deg));

    % 9 – dN/db @ d=0
    [~, iD0] = min(abs(delta_deg));
    M.dN_dBeta_delta0 = mean(gradient(N_mat(:,iD0), beta_deg));

    % 10 – dN/db @ d=dAymax
    M.dN_dBeta_deltaAymax = mean(gradient(N_mat(:,iD_mAy), beta_deg));

    % Store indices for plotting
    M.iB_mAy = iB_mAy;  M.iD_mAy = iD_mAy;
    M.iB_mN  = iB_mN;   M.iD_mN  = iD_mN;
end

%% ── plot_YMD ─────────────────────────────────────────────────────────────
function plot_YMD(V, V_kph, m, a, b, Df, Df_F, Df_R, ...
                  Ay_mat, N_mat, beta_deg, delta_deg, SW_sweep, nB, nD, M)

    cmap = parula(nD);
    fig  = figure('Name', sprintf('YMD  %.0f km/h', V_kph), ...
                  'Color','w', 'Position',[60 60 1800 860]);

    % Manual axis positions: YMD left, colorbar centre-left, table right
    ax  = axes(fig, 'Position', [0.04  0.09  0.55  0.82]);
    axT = axes(fig, 'Position', [0.645 0.04  0.345 0.92]);

    % ── Left: YMD plot ────────────────────────────────────────────────
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');

    % Plot delta lines coloured by SW angle
    for iD = 1:nD
        plot(ax, Ay_mat(:,iD), N_mat(:,iD)/1e3, '-', ...
            'Color', cmap(iD,:), 'LineWidth', 1.8);
    end

    % Beta=0 markers on each delta line
    [~, ib0] = min(abs(beta_deg));
    for iD = 1:nD
        plot(ax, Ay_mat(ib0,iD), N_mat(ib0,iD)/1e3, 'o', ...
            'MarkerFaceColor', cmap(iD,:), 'MarkerEdgeColor','k', 'MarkerSize',5);
    end

    % Constant-beta contours
    betaIdx = round(linspace(1, nB, 13));
    for k = betaIdx
        plot(ax, Ay_mat(k,:), N_mat(k,:)/1e3, '--', ...
            'Color',[0.65 0.65 0.65], 'LineWidth',0.8);
        text(ax, Ay_mat(k,end)*1.008, N_mat(k,end)/1e3, ...
            sprintf('b=%+.0f', beta_deg(k)), 'FontSize',6.5,'Color',[0.4 0.4 0.4]);
    end

    % Key metric markers
    hMaxAy = plot(ax, M.MaxAy, M.N_at_MaxAy/1e3, 'rp', ...
        'MarkerSize',14,'MarkerFaceColor','r','DisplayName','Max Ay');
    hMaxN  = plot(ax, M.Ay_at_MaxN, M.MaxN/1e3, 'g^', ...
        'MarkerSize',11,'MarkerFaceColor','g','DisplayName','Max N');
    hTrim  = plot(ax, M.MaxAy_trimmed, 0, 'ks', ...
        'MarkerSize',11,'MarkerFaceColor','k','DisplayName','Max Ay (trim)');

    text(ax, M.MaxAy+0.01, M.N_at_MaxAy/1e3, ...
        sprintf('  Ay=%.3fg\n  N=%.0fNm', M.MaxAy, M.N_at_MaxAy), ...
        'FontSize',7.5,'Color','r');
    text(ax, M.MaxAy_trimmed+0.01, 0.03, ...
        sprintf('  trim=%.3fg', M.MaxAy_trimmed), 'FontSize',7.5,'Color','k');

    yline(ax,0,'k-','LineWidth',1.0);
    xline(ax,0,'k-','LineWidth',1.0);

    Nmax_k = max(abs(N_mat(:)))/1e3;
    text(ax, 0.25*M.MaxAy,  0.42*Nmax_k, 'UNSTABLE / Loose', ...
        'FontSize',8.5,'Color',[0.7 0.1 0.1],'FontAngle','italic','FontWeight','bold');
    text(ax, 0.25*M.MaxAy, -0.42*Nmax_k, 'STABLE / Push', ...
        'FontSize',8.5,'Color',[0.1 0.5 0.1],'FontAngle','italic','FontWeight','bold');

    xlabel(ax,'Lateral Acceleration  Ay  [g]','FontSize',11);
    ylabel(ax,'Yaw Moment  N  [kN.m]','FontSize',11);
    title(ax, sprintf('Yaw Moment Diagram\nV = %.0f km/h  |  m = %.0f kg  |  Df = %.0f N (F:%.0f R:%.0f)', ...
        V_kph, m, Df, Df_F, Df_R), 'FontSize',10.5,'FontWeight','bold');

    % Marker-only legend (no delta lines)
    legend(ax, [hMaxAy; hMaxN; hTrim], {'Max Ay','Max N','Max Ay (trim)'}, ...
        'Location','northwest','FontSize',8.5,'Box','on');

    % Colourbar for steering wheel angle
    colormap(ax, parula);
    clim(ax, [min(SW_sweep), max(SW_sweep)]);
    cb = colorbar(ax, 'Location','eastoutside');
    cb.Label.String = 'Steering Wheel Angle  SW  [deg]';
    cb.Label.FontSize = 10;
    cb.FontSize = 8.5;
    % Add secondary tick labels showing wheel angle (delta)
    cb.Ticks     = linspace(min(SW_sweep), max(SW_sweep), 9);
    cb.TickLabels = arrayfun(@(sw) sprintf('%.0f (d=%.1f)', sw, ...
        sw / polyval([-4.854730e-10,-7.950252e-22,-1.753110e-05,+4.930866e-18,+4.856089e+00], sw)), ...
        cb.Ticks, 'UniformOutput', false);

    hold(ax,'off');

    % ── Right: metrics table ──────────────────────────────────────────
    set(axT,'Visible','off');

    colX  = [0.01, 0.06, 0.55, 0.72, 0.82];
    Ytop  = 0.97;
    dY    = 0.046;

    % Header bar
    annotation(fig,'rectangle', axPos(axT,[0, Ytop-0.04, 1, 0.04]), ...
        'FaceColor',[0.15 0.22 0.40],'EdgeColor','none');
    hdrLabels = {'#','Metric','Value','Units','Interpretation'};
    for c = 1:5
        annotation(fig,'textbox', axPos(axT,[colX(c), Ytop-0.04, 0.18, 0.04]), ...
            'String',hdrLabels{c},'FontSize',8.5,'FontWeight','bold', ...
            'Color','w','EdgeColor','none','VerticalAlignment','middle','Margin',2);
    end

    % Aero info sub-header
    annotation(fig,'rectangle', axPos(axT,[0, Ytop-0.075, 1, 0.035]), ...
        'FaceColor',[0.93 0.95 0.99],'EdgeColor','none');
    annotation(fig,'textbox', axPos(axT,[0.01, Ytop-0.075, 0.99, 0.035]), ...
        'String', sprintf('Aero: Df = %.0f N  |  Front = %.0f N (%.0f%%)  |  Rear = %.0f N (%.0f%%)', ...
            Df, Df_F, 100*Df_F/max(Df,1), Df_R, 100*Df_R/max(Df,1)), ...
        'FontSize',8,'Color',[0.15 0.25 0.55],'EdgeColor','none', ...
        'VerticalAlignment','middle','Margin',3,'FontWeight','bold','Interpreter','none');

    rows = buildRows(M);
    rowColors = {[0.97 0.97 0.98],[1 1 1]};
    Ystart = Ytop - 0.075;          % top of data rows (below header + aero bar)
    Ybottom = 0.01;                 % minimum y before clipping
    nRows = numel(rows);
    % Auto-fit row height so all rows fit within [Ybottom, Ystart]
    dY = (Ystart - Ybottom) / nRows;
    dY = min(dY, 0.046);            % cap at comfortable max height

    for r = 1:nRows
        row  = rows{r};
        Yrow = Ystart - r*dY;

        % Skip any row that would fall outside [0,1] — safety guard
        if Yrow < 0 || (Yrow + dY) > 1; continue; end

        if strcmp(row.type,'section')
            annotation(fig,'rectangle', axPos(axT,[0, Yrow, 1, dY]), ...
                'FaceColor',[0.87 0.90 0.96],'EdgeColor','none');
            annotation(fig,'textbox', axPos(axT,[0.01, Yrow, 0.99, dY]), ...
                'String',upper(row.label),'FontSize',7.5,'FontWeight','bold', ...
                'Color',[0.15 0.25 0.55],'EdgeColor','none', ...
                'VerticalAlignment','middle','Margin',3,'Interpreter','none');
            continue
        end

        bgc = rowColors{mod(r,2)+1};
        annotation(fig,'rectangle', axPos(axT,[0, Yrow, 1, dY]), ...
            'FaceColor',bgc,'EdgeColor',[0.84 0.84 0.87]);

        cells   = {row.num, row.metric, row.value, row.units, row.interp};
        fcolors = {[0.50 0.50 0.50],[0.05 0.05 0.05],[0.05 0.35 0.65], ...
                   [0.40 0.40 0.40], interpColor(row.interp)};
        fweights= {'normal','bold','normal','normal','normal'};
        fsizes  = {7.5, 8, 8.5, 7.5, 7.5};
        for c = 1:5
            annotation(fig,'textbox', axPos(axT,[colX(c), Yrow, 0.18, dY]), ...
                'String',cells{c},'FontSize',fsizes{c},'FontWeight',fweights{c}, ...
                'Color',fcolors{c},'EdgeColor','none', ...
                'VerticalAlignment','middle','Margin',2,'Interpreter','none');
        end
    end

    % Panel title
    annotation(fig,'textbox', axPos(axT,[0, Ytop, 1, 0.03]), ...
        'String','YMD Metrics Table','FontSize',10.5,'FontWeight','bold', ...
        'Color',[0.05 0.05 0.05],'EdgeColor','none', ...
        'VerticalAlignment','middle','Margin',2,'Interpreter','none');

    sgtitle(sprintf('YMD Analysis  |  V = %.0f km/h  |  m = %.0f kg  |  MF6.1 Tyre', ...
        V_kph, m), 'FontSize',13,'FontWeight','bold');

    % Figures displayed interactively; no files saved.
end

%% ── buildRows ────────────────────────────────────────────────────────────
function rows = buildRows(M)
    function r = sec(lbl); r = struct('type','section','label',lbl); end
    function r = dat(num,metric,value,units,interp)
        r = struct('type','data','num',num,'metric',metric, ...
                   'value',value,'units',units,'interp',interp);
    end

    v1  = sprintf('%.3f',  M.MaxAy);
    v2  = sprintf('%.3f',  M.MaxAy_trimmed);
    gap = sprintf('dAy = %.3f g', M.MaxAy - M.MaxAy_trimmed);
    v3  = sprintf('%.1f',  M.N_at_MaxAy);
    v4a = sprintf('%.2f',  M.alphaF_at_MaxAy);
    v4b = sprintf('%.2f',  M.alphaR_at_MaxAy);
    v4c = sprintf('%.2f',  M.beta_at_MaxAy);
    v4d  = sprintf('%.2f',  M.delta_at_MaxAy);
    v4d_sw = sprintf('(SW = %.1f deg)', M.SW_at_MaxAy);
    v4e = sprintf('%.1f',  M.FyF_at_MaxAy);
    v4f = sprintf('%.1f',  M.FyR_at_MaxAy);
    v5  = sprintf('%.1f',  M.MaxN);
    v6  = sprintf('%.1f / %.1f / %.1f / %.1f', ...
                  M.alphaF_at_MaxN, M.alphaR_at_MaxN, M.beta_at_MaxN, M.delta_at_MaxN);
    i6  = sprintf('Ay = %.3f g', M.Ay_at_MaxN);
    v7  = sprintf('%.1f',  M.dN_dDelta_beta0);
    v8  = sprintf('%.1f',  M.dN_dDelta_betaAymax);
    v9  = sprintf('%.1f',  M.dN_dBeta_delta0);
    v10 = sprintf('%.1f',  M.dN_dBeta_deltaAymax);

    rows = {
        sec('Performance envelope')
        dat('1',  'Max lateral acceleration',       v1,  'g',       'Peak cornering envelope')
        dat('2',  'Max Ay (trimmed, N~0)',           v2,  'g',       gap)
        sec('Stability / neutral-steer indicator')
        dat('3',  'Yaw moment N @ max Ay',           v3,  'N.m',    condStr(M.N_at_MaxAy<0,'STABLE / Push','UNSTABLE / Loose'))
        sec('State at max lateral acceleration')
        dat('4a', 'Front slip angle aF',             v4a, 'deg',    'Front tyre operating point')
        dat('4b', 'Rear slip angle aR',              v4b, 'deg',    'Rear tyre operating point')
        dat('4c', 'Body slip angle beta',            v4c, 'deg',    'CG attitude at limit')
        dat('4d', 'Wheel steer angle delta',         v4d, 'deg',    v4d_sw)
        dat('4e', 'Front cornering force FyF',       v4e, 'N',      'Front axle lateral force')
        dat('4f', 'Rear cornering force FyR',        v4f, 'N',      'Rear axle lateral force')
        sec('Maximum yaw moment')
        dat('5',  'Max yaw moment |N|',              v5,  'N.m',   'Available rotation torque')
        dat('6',  'aF/aR/beta/delta @ max N',        v6,  'deg',   i6)
        sec('Controllability  dN/d(delta)  [delta = wheel steer angle]')
        dat('7',  'dN/d(delta) @ beta=0  (entry)',         v7,  'Nm/deg', condStr(M.dN_dDelta_beta0>0,    '+ve: steering effective',   '-ve: steering ineffective'))
        dat('8',  'dN/d(delta) @ beta=bAymax  (apex)',     v8,  'Nm/deg', condStr(M.dN_dDelta_betaAymax>0,'+ve: controllable at limit','-ve: front saturated'))
        sec('Stability  dN/d(beta)  [beta = body slip angle]')
        dat('9',  'dN/d(beta) @ delta=0  (entry)',        v9,  'Nm/deg', condStr(M.dN_dBeta_delta0<0,    '-ve: restoring (stable)',   '+ve: diverging (unstable)'))
        dat('10', 'dN/d(beta) @ delta=dAymax  (apex)',    v10, 'Nm/deg', condStr(M.dN_dBeta_deltaAymax<0,'-ve: stable at apex',      '+ve: unstable at apex'))
    };
end

%% ── printMetricsTable ────────────────────────────────────────────────────
function printMetricsTable(M, V, m, a, b, Fz_F, Fz_R, Df, Df_F, Df_R)
    sep = repmat('-',1,78);
    fprintf('\n%s\n', sep);
    fprintf('  YMD METRICS  |  V = %.0f m/s (%.0f km/h)  |  m = %.0f kg\n', V, V*3.6, m);
    fprintf('  Aero: Df = %.1f N  (Front %.1f N + Rear %.1f N)\n', Df, Df_F, Df_R);
    fprintf('  Fz_F = %.1f N  |  Fz_R = %.1f N  |  a=%.2fm  b=%.2fm\n', Fz_F, Fz_R, a, b);
    fprintf('%s\n', sep);
    fprintf('  1  Max Ay                    %+.3f g\n',   M.MaxAy);
    fprintf('  2  Max Ay (trimmed N~0)      %+.3f g   (gap %.3f g)\n', M.MaxAy_trimmed, M.MaxAy-M.MaxAy_trimmed);
    fprintf('  3  N @ Max Ay               %+.1f N.m  => %s\n', M.N_at_MaxAy, condStr(M.N_at_MaxAy<0,'STABLE/Push','UNSTABLE/Loose'));
    fprintf('  4  aF/aR/b/d @ MaxAy:  %.2f / %.2f / %.2f / %.2f deg\n', ...
            M.alphaF_at_MaxAy, M.alphaR_at_MaxAy, M.beta_at_MaxAy, M.delta_at_MaxAy);
    fprintf('     SW @ MaxAy = %.1f deg\n', M.SW_at_MaxAy);
    fprintf('     FyF = %.1f N   FyR = %.1f N\n', M.FyF_at_MaxAy, M.FyR_at_MaxAy);
    fprintf('  5  Max |N|                  %+.1f N.m\n', M.MaxN);
    fprintf('  6  @ Max N: Ay=%.3fg  aF=%.2f  aR=%.2f  b=%.2f  d=%.2f deg\n', ...
            M.Ay_at_MaxN, M.alphaF_at_MaxN, M.alphaR_at_MaxN, M.beta_at_MaxN, M.delta_at_MaxN);
    fprintf('  7  dN/d(delta) @ beta=0       %+.1f Nm/deg  => %s\n', M.dN_dDelta_beta0,      condStr(M.dN_dDelta_beta0>0,   'steering effective','steering ineffective'));
    fprintf('  8  dN/d(delta) @ beta=bAymax  %+.1f Nm/deg  => %s\n', M.dN_dDelta_betaAymax,  condStr(M.dN_dDelta_betaAymax>0,'controllable at limit','FRONT SAT'));
    fprintf('  9  dN/d(beta) @ delta=0       %+.1f Nm/deg  => %s\n', M.dN_dBeta_delta0,      condStr(M.dN_dBeta_delta0<0,  'restoring (stable)','diverging (unstable)'));
    fprintf(' 10  dN/d(beta) @ delta=dAymax  %+.1f Nm/deg  => %s\n', M.dN_dBeta_deltaAymax,  condStr(M.dN_dBeta_deltaAymax<0,'stable at apex','unstable at apex'));
    fprintf('%s\n', sep);
end

%% ── axPos ────────────────────────────────────────────────────────────────
function fp = axPos(ax, relRect)
    ap = ax.Position;
    fp = [ap(1)+relRect(1)*ap(3), ap(2)+relRect(2)*ap(4), ...
          relRect(3)*ap(3),        relRect(4)*ap(4)];
end

%% ── interpColor ──────────────────────────────────────────────────────────
function c = interpColor(str)
    if contains(str,{'+ve','STABLE','effective','controllable','stable'})
        c = [0.05 0.42 0.10];
    elseif contains(str,{'-ve','UNSTABLE','saturated','ineffective','unstable','Loose'})
        c = [0.65 0.08 0.08];
    else
        c = [0.25 0.25 0.25];
    end
end

%% ── condStr ──────────────────────────────────────────────────────────────
function s = condStr(cond, trueStr, falseStr)
    if cond; s = trueStr; else; s = falseStr; end
end

%% ── parseTIR ─────────────────────────────────────────────────────────────
function tirParams = parseTIR(filename)
    tirParams = struct();
    fid = fopen(filename,'r');
    if fid < 0; error('Cannot open: %s', filename); end
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line) || ismember(line(1), {'$','[','!'}); continue; end
        parts = strsplit(line,'=');
        if numel(parts) < 2; continue; end
        key    = strtrim(parts{1});
        valStr = strtrim(regexp(strtrim(parts{2}),'\$.*','split'));
        val    = str2double(valStr{1});
        if ~isnan(val); tirParams.(key) = val; end
    end
    fclose(fid);
end