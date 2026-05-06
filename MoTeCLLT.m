%% LATERAL LOAD TRANSFER FROM MOTEC I2PRO DATA
clear; clc; close all;

% =========================================================================
%% LOAD DATA & PARAMETERS
% =========================================================================
motec = load('MoTeC data/JTCendurance_test9.mat');
paramR26;

% =========================================================================
%% COMPUTE LLT
% =========================================================================
[t, TLTf, TLTr, Total_LLT, roll_angle, front_roll_out, rear_roll_out] = computeVehicleLLT();

static_load   = (198 + 66) * 9.81 / 4;
average_TLT_f = mean(TLTf);
average_TLT_r = mean(TLTr);

fprintf('=== LATERAL LOAD TRANSFER SUMMARY ===\n');
fprintf('Static Corner Load  : %.2f N\n',   static_load);
fprintf('Mean Front LLT      : %.2f N\n',   average_TLT_f);
fprintf('Mean Rear  LLT      : %.2f N\n\n', average_TLT_r);

% =========================================================================
%% THEORETICAL TOTAL LLT (VERIFICATION)
% =========================================================================
lateral_g_interp  = interp1(motec.Average_Lat_G.Time, motec.Average_Lat_G.Value, ...
                             t, 'linear', 'extrap');
theoretical_total = (lateral_g_interp * 9.81 * car.m * car.cgh) / car.track;

% =========================================================================
%% DAMPER LINEAR VELOCITY
% =========================================================================
half_track = car.track / 2;
dphi_f     = gradient(front_roll_out, t);
dphi_r     = gradient(rear_roll_out,  t);
v_damper_f = dphi_f * half_track * front.MR * 1000;
v_damper_r = dphi_r * half_track * rear.MR  * 1000;

% =========================================================================
%% DARK THEME HELPER
% =========================================================================
function style_dark(ax)
    set(ax, 'Color', 'k', ...
            'XColor', 'w', 'YColor', 'w', ...
            'GridColor', [0.4 0.4 0.4], ...
            'MinorGridColor', [0.25 0.25 0.25]);
end

function fig = dark_figure(num)
    fig = figure(num);
    set(fig, 'Color', 'k');
end

% =========================================================================
%% PLOTS
% =========================================================================

% --- Figure 1: Front, Rear & Total LLT ---
dark_figure(1);
ax = axes;
plot(ax, t, TLTf,      'r', 'LineWidth', 1.5, 'DisplayName', 'Front LLT'); hold on;
plot(ax, t, TLTr,      'b', 'LineWidth', 1.5, 'DisplayName', 'Rear LLT');
plot(ax, t, Total_LLT, 'g', 'LineWidth', 1.5, 'DisplayName', 'Total LLT');
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Lateral Load Transfer — Raw Data', 'Color', 'w');
lg = legend; set(lg, 'TextColor', 'w', 'Color', 'k', 'EdgeColor', 'w');
grid on; style_dark(ax);

% --- Figure 2: Theoretical vs Computed Total LLT ---
dark_figure(2);
ax = axes;
plot(ax, t, theoretical_total, 'r', 'LineWidth', 1.5, 'DisplayName', 'Theoretical Total'); hold on;
plot(ax, t, Total_LLT,         'b', 'LineWidth', 1.5, 'DisplayName', 'Computed Total');
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Total LLT Verification', 'Color', 'w');
lg = legend; set(lg, 'TextColor', 'w', 'Color', 'k', 'EdgeColor', 'w');
grid on; style_dark(ax);

% --- Figure 3: Roll Angles ---
dark_figure(3);
ax = axes;
plot(ax, t, front_roll_out * 180/pi, 'r', 'LineWidth', 1.5, 'DisplayName', 'Front'); hold on;
plot(ax, t, rear_roll_out  * 180/pi, 'b', 'LineWidth', 1.5, 'DisplayName', 'Rear');
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Roll Angles', 'Color', 'w');
lg = legend; set(lg, 'TextColor', 'w', 'Color', 'k', 'EdgeColor', 'w');
grid on; style_dark(ax);

% --- Figure 4: Damper Linear Velocity & Distribution ---
dark_figure(4);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Damper Linear Velocity from Roll', 'Color', 'w');

nexttile(1); ax = gca;
plot(t, v_damper_f, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (mm/s)');
title('Front Damper Velocity', 'Color', 'w');
grid on; style_dark(ax);

nexttile(2); ax = gca;
counts_f = histcounts(v_damper_f, 50);
pct_f    = counts_f / sum(counts_f) * 100;
edges_f  = linspace(min(v_damper_f), max(v_damper_f), 51);
bar(edges_f(1:end-1), pct_f, 1, 'FaceColor', 'r', 'EdgeColor', 'none');
xlabel('Velocity (mm/s)'); ylabel('Occurrence (%)');
title('Front Velocity Distribution', 'Color', 'w');
grid on; style_dark(ax);

nexttile(3); ax = gca;
plot(t, v_damper_r, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (mm/s)');
title('Rear Damper Velocity', 'Color', 'w');
grid on; style_dark(ax);

nexttile(4); ax = gca;
counts_r = histcounts(v_damper_r, 50);
pct_r    = counts_r / sum(counts_r) * 100;
edges_r  = linspace(min(v_damper_r), max(v_damper_r), 51);
bar(edges_r(1:end-1), pct_r, 1, 'FaceColor', 'b', 'EdgeColor', 'none');
xlabel('Velocity (mm/s)'); ylabel('Occurrence (%)');
title('Rear Velocity Distribution', 'Color', 'w');
grid on; style_dark(ax);