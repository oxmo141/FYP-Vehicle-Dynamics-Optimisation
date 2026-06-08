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

static_load_front   = (car.m * weight_distribution)/2;
static_load_rear = (car.m * (1-weight_distribution))/2;
average_TLT_f = mean(TLTf);
average_TLT_r = mean(TLTr);

fprintf('=== LATERAL LOAD TRANSFER SUMMARY ===\n');
fprintf('Static Corner Load Front : %.2f N\n', static_load_front);
fprintf('Static Corner Load Rear  : %.2f N\n', static_load_rear);
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
    set(ax, 'Color', 'w', ...
            'XColor', 'k', 'YColor', 'k', ...
            'GridColor', [0.6 0.6 0.6], ...
            'MinorGridColor', [0.75 0.75 0.75]);
end

function fig = dark_figure(num)
    fig = figure(num);
    set(fig, 'Color', 'w');
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
title('Lateral Load Transfer — Raw Data', 'Color', 'k');
lg = legend; set(lg, 'TextColor', 'k', 'Color', 'w', 'EdgeColor', 'k');
grid on; style_dark(ax);

% --- Figure 2: Theoretical vs Computed Total LLT ---
dark_figure(2);
ax = axes;
plot(ax, t, theoretical_total, 'r', 'LineWidth', 1.5, 'DisplayName', 'Theoretical Total'); hold on;
plot(ax, t, Total_LLT,         'b', 'LineWidth', 1.5, 'DisplayName', 'Computed Total');
xlabel('Time (s)'); ylabel('Load Transfer (N)');
title('Total LLT Verification', 'Color', 'k');
lg = legend; set(lg, 'TextColor', 'k', 'Color', 'w', 'EdgeColor', 'k');
grid on; style_dark(ax);

% --- Figure 3: Roll Angles ---
dark_figure(3);
ax = axes;
plot(ax, t, front_roll_out * 180/pi, 'r', 'LineWidth', 1.5, 'DisplayName', 'Front'); hold on;
plot(ax, t, rear_roll_out  * 180/pi, 'b', 'LineWidth', 1.5, 'DisplayName', 'Rear');
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Roll Angles', 'Color', 'k');
lg = legend; set(lg, 'TextColor', 'k', 'Color', 'w', 'EdgeColor', 'k');
grid on; style_dark(ax);

% --- Figure 4: Damper Linear Velocity & Distribution ---
dark_figure(4);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Damper Linear Velocity from Roll', 'Color', 'k');

nexttile(1); ax = gca;
plot(t, v_damper_f, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (mm/s)');
title('Front Damper Velocity', 'Color', 'k');
grid on; style_dark(ax);

nexttile(2); ax = gca;
counts_f = histcounts(v_damper_f, 50);
pct_f    = counts_f / sum(counts_f) * 100;
edges_f  = linspace(min(v_damper_f), max(v_damper_f), 51);
bar(edges_f(1:end-1), pct_f, 1, 'FaceColor', 'r', 'EdgeColor', 'none');
xlabel('Velocity (mm/s)'); ylabel('Occurrence (%)');
title('Front Velocity Distribution', 'Color', 'k');
grid on; style_dark(ax);

nexttile(3); ax = gca;
plot(t, v_damper_r, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (mm/s)');
title('Rear Damper Velocity', 'Color', 'k');
grid on; style_dark(ax);

nexttile(4); ax = gca;
counts_r = histcounts(v_damper_r, 50);
pct_r    = counts_r / sum(counts_r) * 100;
edges_r  = linspace(min(v_damper_r), max(v_damper_r), 51);
bar(edges_r(1:end-1), pct_r, 1, 'FaceColor', 'b', 'EdgeColor', 'none');
xlabel('Velocity (mm/s)'); ylabel('Occurrence (%)');
title('Rear Velocity Distribution', 'Color', 'k');
grid on; style_dark(ax);

% --- Figure 5+6: Statistics Table + Distribution Analysis ---
% --- Compute distribution metrics (used in combined figure) ---
skew_f = skewness(v_damper_f);
skew_r = skewness(v_damper_r);
kurt_f = kurtosis(v_damper_f);
kurt_r = kurtosis(v_damper_r);

warning('off', 'stats:lillietest:OutOfRangePLow');
[h_f, p_f] = lillietest(v_damper_f);
[h_r, p_r] = lillietest(v_damper_r);
warning('on',  'stats:lillietest:OutOfRangePLow');

if p_f <= 0.001, p_f_str = 'p < 0.001'; else, p_f_str = sprintf('p = %.4f', p_f); end
if p_r <= 0.001, p_r_str = 'p < 0.001'; else, p_r_str = sprintf('p = %.4f', p_r); end

if h_f == 0, norm_f = 'Normal'; else, norm_f = 'Non-normal'; end
if h_r == 0, norm_r = 'Normal'; else, norm_r = 'Non-normal'; end

skews  = [skew_f,  skew_r];
kurts  = [kurt_f,  kurt_r];
p_strs = {p_f_str, p_r_str};
norms  = {norm_f,  norm_r};

fprintf('=== DISTRIBUTION ANALYSIS ===\n');
fprintf('             Front          Rear\n');
fprintf('Skewness   : %+.4f        %+.4f\n', skew_f, skew_r);
fprintf('Kurtosis   : %.4f         %.4f\n',   kurt_f, kurt_r);
fprintf('Lilliefors : %s            %s\n',    p_f_str, p_r_str);
fprintf('Distribution: %s           %s\n',   norm_f,  norm_r);

% --- Compute velocity statistics (used in combined figure) ---
stats.mean_f   = mean(v_damper_f);
stats.std_f    = std(v_damper_f);
stats.lower5_f = prctile(v_damper_f, 5);
stats.upper5_f = prctile(v_damper_f, 95);

stats.mean_r   = mean(v_damper_r);
stats.std_r    = std(v_damper_r);
stats.lower5_r = prctile(v_damper_r, 5);
stats.upper5_r = prctile(v_damper_r, 95);

dark_figure(5);
tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Damper Velocity — Statistics & Distribution Analysis', 'Color', 'k');

% --- Tile 1: Front distribution ---
nexttile(1); ax = gca;
histogram(v_damper_f, 50, 'Normalization', 'pdf', ...
    'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on;
[f_kde, xi] = ksdensity(v_damper_f);
plot(xi, f_kde, 'k-', 'LineWidth', 2, 'DisplayName', 'KDE');
mu_fit = mean(v_damper_f); sigma_fit = std(v_damper_f);
x_fit  = linspace(min(v_damper_f), max(v_damper_f), 300);
plot(x_fit, normpdf(x_fit, mu_fit, sigma_fit), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Normal fit');
annotation_str = sprintf('Skewness     : %+.3f\nKurtosis     : %.3f\nLilliefors : %s\nDistribution : %s', ...
    skews(1), kurts(1), p_strs{1}, norms{1});
text(0.97, 0.97, annotation_str, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'Interpreter', 'none');
xlabel('Velocity (mm/s)'); ylabel('Probability Density');
title('Front Damper — Distribution Fit', 'Color', 'k');
lg = legend('Histogram', 'KDE', 'Normal fit');
set(lg, 'TextColor', 'k', 'Color', 'w', 'EdgeColor', 'k');
grid on; style_dark(ax);

% --- Tile 2: Rear distribution ---
nexttile(2); ax = gca;
histogram(v_damper_r, 50, 'Normalization', 'pdf', ...
    'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on;
[f_kde, xi] = ksdensity(v_damper_r);
plot(xi, f_kde, 'k-', 'LineWidth', 2, 'DisplayName', 'KDE');
mu_fit = mean(v_damper_r); sigma_fit = std(v_damper_r);
x_fit  = linspace(min(v_damper_r), max(v_damper_r), 300);
plot(x_fit, normpdf(x_fit, mu_fit, sigma_fit), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Normal fit');
annotation_str = sprintf('Skewness     : %+.3f\nKurtosis     : %.3f\nLilliefors : %s\nDistribution : %s', ...
    skews(2), kurts(2), p_strs{2}, norms{2});
text(0.97, 0.97, annotation_str, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'Interpreter', 'none');
xlabel('Velocity (mm/s)'); ylabel('Probability Density');
title('Rear Damper — Distribution Fit', 'Color', 'k');
lg = legend('Histogram', 'KDE', 'Normal fit');
set(lg, 'TextColor', 'k', 'Color', 'w', 'EdgeColor', 'k');
grid on; style_dark(ax);

% --- Tile 3: Statistics table ---
nexttile(3); ax = gca; axis(ax, 'off');
title(ax, 'Velocity Statistics', 'Color', 'k', 'FontSize', 11, 'FontWeight', 'bold');

row_data = {
    'Static Corner Load',  sprintf('%.2f', static_load_front), sprintf('%.2f', static_load_rear);
    'Mean',                sprintf('%.2f', stats.mean_f),      sprintf('%.2f', stats.mean_r);
    'Std Dev',             sprintf('%.2f', stats.std_f),       sprintf('%.2f', stats.std_r);
    '5th Percentile',      sprintf('%.2f', stats.lower5_f),    sprintf('%.2f', stats.lower5_r);
    '95th Percentile',     sprintf('%.2f', stats.upper5_f),    sprintf('%.2f', stats.upper5_r);
    'Skewness',            sprintf('%+.3f', skews(1)),         sprintf('%+.3f', skews(2));
    'Kurtosis',            sprintf('%.3f',  kurts(1)),         sprintf('%.3f',  kurts(2));
    'Lilliefors',          p_strs{1},                          p_strs{2};
    'Distribution',        norms{1},                           norms{2};
};

col_headers = {'Statistic', 'Front (mm/s)', 'Rear (mm/s)'};
t_data = [col_headers; row_data];

uitable(gcf, ...
    'Data',            t_data, ...
    'Units',           'normalized', ...
    'Position',        [0.675 0.1 0.30 0.75], ...
    'ColumnWidth',     {130, 90, 90}, ...
    'RowName',         [], ...
    'ColumnName',      [], ...
    'FontSize',        10, ...
    'BackgroundColor', [1 1 1; 0.94 0.94 0.94], ...
    'RowStriping',     'on');