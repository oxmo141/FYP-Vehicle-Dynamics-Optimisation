%% damper_dyno_bilinear.m
% Damper Dyno Analysis — Bi-linear Damping Coefficient Fit
%
% Filename convention:  [WHL] [HS_comp] [HS_reb] [LS_comp] [LS_reb].dctw
%   WHL     = wheel identifier (first 2 chars, e.g. FR, FL, RR, RL)
%   HS_comp = high-speed compression adjuster clicks  (1st number)
%   HS_reb  = high-speed rebound  adjuster clicks     (2nd number)
%   LS_comp = low-speed  compression adjuster clicks  (3rd number)
%   LS_reb  = low-speed  rebound  adjuster clicks     (4th number)
%
% Sign convention:
%   Compression stroke → velocity > 0,  force > 0
%   Rebound stroke     → velocity < 0,  force < 0
%   All F-V curves are plotted with |force| vs |velocity|
%
% Bi-linear model:
%   F(v) = C_ls * v                           for  v <= V_knee
%   F(v) = F_knee + C_hs * (v - V_knee)      for  v >  V_knee
%   (F_knee = C_ls * V_knee — continuity enforced)
%
% Outputs:
%   • F vs |V| plot with bi-linear fits overlaid for all runs
%   • Bar chart: C_ls and C_hs per run, compression & rebound
%   • Printed summary table in the Command Window
%   • Saved PNG figure
%
% Requirements: MATLAB R2016b+ — no extra toolboxes needed.

clear; clc; close all;

%% ── Configuration ────────────────────────────────────────────────────────
MAT_FILE      = 'RR_25_26_high_speed_sweep.mat';
N_BINS        = 60;     % velocity bins for F-V extraction
VEL_THRESHOLD = 5;      % mm/s  — ignore data within this band of zero
PERCENTILE    = 90;     % robust peak: use top/bottom N% of forces per bin

%% ── Colour palette (one per run, up to 12) ───────────────────────────────
CMAP = [
    0.839 0.153 0.157;   % red
    0.122 0.471 0.706;   % blue
    1.000 0.498 0.055;   % orange
    0.173 0.627 0.173;   % green
    0.580 0.404 0.741;   % purple
    0.100 0.100 0.100;   % black
    0.549 0.337 0.294;   % brown
    0.890 0.467 0.761;   % pink
    0.498 0.498 0.498;   % grey
    0.737 0.741 0.133;   % yellow-green
    0.090 0.745 0.812;   % cyan
    0.682 0.780 0.910;   % light blue
];

%% ── Wheel label lookup ───────────────────────────────────────────────────
WHEEL_LABELS = containers.Map( ...
    {'FR','FL','RR','RL'}, ...
    {'Front Right','Front Left','Rear Right','Rear Left'});

%% ── Load data ────────────────────────────────────────────────────────────
fprintf('Loading: %s\n', MAT_FILE);
S = load(MAT_FILE);
if isfield(S, 'files')
    files = S.files;
else
    fn = fieldnames(S); files = S.(fn{1});
    warning('Variable ''files'' not found; using ''%s''.', fn{1});
end
n_runs = numel(files);
fprintf('  Found %d run(s).\n\n', n_runs);

% Extend colour map if needed
if n_runs > size(CMAP,1)
    CMAP = [CMAP; lines(n_runs - size(CMAP,1))];
end

%% ── Pre-allocate results storage ─────────────────────────────────────────
res(n_runs) = struct( ...
    'fname','', 'label','', 'short','', ...
    'wheel_code','', 'wheel_long','', ...
    'hs_c',0,'hs_r',0,'ls_c',0,'ls_r',0, ...
    'cv',[],'cf',[],'rv',[],'rf',[], ...
    'c_ls_c',0,'c_hs_c',0,'vk_c',0,'fk_c',0,'r2_c',0, ...
    'c_ls_r',0,'c_hs_r',0,'vk_r',0,'fk_r',0,'r2_r',0);

%% ── Process each run ─────────────────────────────────────────────────────
for i = 1:n_runs

    % ── Extract velocity & force ─────────────────────────────────────────
    sig_struct = files(i).Data.signals;
    vel = []; force = [];
    for j = 1:numel(sig_struct)
        switch sig_struct(j).Name
            case 'Velocity'; vel   = double(sig_struct(j).Value(:)');
            case 'Force';    force = double(sig_struct(j).Value(:)');
        end
    end
    if isempty(vel) || isempty(force)
        warning('Run %d: missing signals — skipped.', i); continue
    end

    % ── Sign convention diagnostic (raw ranges, before any processing) ────
    fprintf('  Run %d sign check:\n', i);
    fprintf('    vel>0 samples: force in [%+.1f, %+.1f] N\n', ...
            min(force(vel >  50)), max(force(vel >  50)));
    fprintf('    vel<0 samples: force in [%+.1f, %+.1f] N\n', ...
            min(force(vel < -50)), max(force(vel < -50)));

    % ── Parse filename → wheel code + adjuster settings ───────────────────
    fname = files(i).File;
    [~, bname] = fileparts(fname);

    % Extract wheel code: first 2 uppercase letters
    wheel_match = regexp(bname, '^([A-Za-z]{2})', 'tokens', 'once');
    if ~isempty(wheel_match)
        wheel_code = upper(wheel_match{1});
    else
        wheel_code = 'XX';
    end
    if isKey(WHEEL_LABELS, wheel_code)
        wheel_long = WHEEL_LABELS(wheel_code);
    else
        wheel_long = wheel_code;
    end

    tokens = regexp(bname, '\d+', 'match');
    hs_c = str2double(tokens{1});
    hs_r = str2double(tokens{2});
    ls_c = str2double(tokens{3});
    ls_r = str2double(tokens{4});

    res(i).fname      = fname;
    res(i).wheel_code = wheel_code;
    res(i).wheel_long = wheel_long;
    res(i).label = sprintf('%s — HSc%d/HSr%d LSc%d/LSr%d', ...
                            wheel_code, hs_c, hs_r, ls_c, ls_r);
    res(i).short = sprintf('%s\nHSc%d/HSr%d', wheel_code, hs_c, hs_r);
    res(i).hs_c  = hs_c;  res(i).hs_r = hs_r;
    res(i).ls_c  = ls_c;  res(i).ls_r = ls_r;

    % ── Bin into F-V curve ────────────────────────────────────────────────
    [cv, cf, rv, rf] = extract_fv_curve(vel, force, N_BINS, VEL_THRESHOLD, PERCENTILE);
    res(i).cv = cv;  res(i).cf = cf;
    res(i).rv = rv;  res(i).rf = rf;

    % ── Fit bi-linear model ───────────────────────────────────────────────
    [c_ls_c, c_hs_c, vk_c, fk_c, r2_c] = fit_bilinear(cv, cf);
    [c_ls_r, c_hs_r, vk_r, fk_r, r2_r] = fit_bilinear(rv, rf);

    res(i).c_ls_c = c_ls_c;  res(i).c_hs_c = c_hs_c;
    res(i).vk_c   = vk_c;    res(i).fk_c   = fk_c;    res(i).r2_c = r2_c;
    res(i).c_ls_r = c_ls_r;  res(i).c_hs_r = c_hs_r;
    res(i).vk_r   = vk_r;    res(i).fk_r   = fk_r;    res(i).r2_r = r2_r;
end

%% ── Determine overall wheel label for figure titles ──────────────────────
all_wheel_codes = unique({res.wheel_code}, 'stable');
all_wheel_longs = unique({res.wheel_long}, 'stable');
if numel(all_wheel_codes) == 1
    wheel_code_all = all_wheel_codes{1};
    wheel_long_all = all_wheel_longs{1};
    WHEEL_PREFIX   = sprintf('[%s — %s]', wheel_code_all, wheel_long_all);
else
    WHEEL_PREFIX = sprintf('[%s]', strjoin(all_wheel_codes, ' / '));
end

%% ── Print summary table ──────────────────────────────────────────────────
fprintf('\n%s\n', repmat('─',1,100));
fprintf('%-28s │ %36s │ %36s\n', ...
    sprintf('  %s Setting', WHEEL_PREFIX), ...
    '         COMPRESSION', '            REBOUND');
fprintf('%-28s │ %36s │ %36s\n', ...
    '  (HSc / HSr / LSc / LSr)', ...
    'C_ls(N·s/mm)  V_knee(mm/s)  C_hs(N·s/mm)     R²', ...
    'C_ls(N·s/mm)  V_knee(mm/s)  C_hs(N·s/mm)     R²');
fprintf('%s\n', repmat('─',1,100));
for i = 1:n_runs
    r = res(i);
    fprintf('  HSc%-2d / HSr%-2d / LSc%-2d / LSr%-2d │ %12.3f  %12.1f  %12.3f  %5.4f │ %12.3f  %12.1f  %12.3f  %5.4f\n', ...
        r.hs_c, r.hs_r, r.ls_c, r.ls_r, ...
        r.c_ls_c, r.vk_c, r.c_hs_c, r.r2_c, ...
        r.c_ls_r, r.vk_r, r.c_hs_r, r.r2_r);
end
fprintf('%s\n\n', repmat('─',1,100));
fprintf('Units: C_ls, C_hs in N·s/mm = kN·s/m.  V_knee in mm/s.\n');
fprintf('Bi-linear model: F = C_ls·v (v ≤ V_knee); F = F_knee + C_hs·(v−V_knee) (v > V_knee)\n\n');

%% ── Figure 1: plots (2×2 grid) ───────────────────────────────────────────
TITLE_STR = sprintf('%s  High Speed Sweep — Bi-linear Damping Coefficient Analysis', ...
                    WHEEL_PREFIX);

fig = figure('Name', sprintf('%s Damper Dyno Analysis', WHEEL_PREFIX), ...
             'Color','w', 'Position', [30 30 1600 820]);

annotation(fig, 'textbox', [0 0.955 1 0.04], ...
    'String', TITLE_STR, 'FontSize', 14, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'EdgeColor', 'none', ...
    'FitBoxToText', 'off', 'BackgroundColor', 'none', 'Color', 'k');

% ── Row 1: F-V plots ──────────────────────────────────────────────────────
ax_fv_c = subplot('Position', [0.05  0.55  0.42  0.38]);
ax_fv_r = subplot('Position', [0.55  0.55  0.42  0.38]);

axes_list  = {ax_fv_c, ax_fv_r};
curve_tags = {'Compression','Rebound'};
legend_h   = gobjects(n_runs,1);

for side = 1:2
    ax = axes_list{side};
    hold(ax,'on'); grid(ax,'on');
    ax.Color           = 'w';
    ax.GridLineStyle   = '--';
    ax.GridAlpha       = 0.25;
    ax.GridColor       = [0.55 0.55 0.55];
    ax.MinorGridColor  = [0.75 0.75 0.75];
    ax.XColor          = 'k';
    ax.YColor          = 'k';
    ax.FontSize        = 10;
    ax.Box             = 'on';
    ax.BoxStyle        = 'back';

    for i = 1:n_runs
        r   = res(i);
        col = CMAP(i,:);
        if side == 1
            vd = r.cv; fd = r.cf;
            c_ls = r.c_ls_c; c_hs = r.c_hs_c; vk = r.vk_c; fk = r.fk_c;
        else
            vd = r.rv; fd = r.rf;
            c_ls = r.c_ls_r; c_hs = r.c_hs_r; vk = r.vk_r; fk = r.fk_r;
        end
        plot(ax, vd, fd, 'o', 'Color', col, 'MarkerSize', 3, ...
             'MarkerFaceColor', col, 'MarkerEdgeColor','none', 'HandleVisibility','off');
        v_fit = linspace(0, max(vd)*1.03, 400);
        f_fit = bilinear_eval(v_fit, c_ls, c_hs, vk, fk);
        h = plot(ax, v_fit, f_fit, '-', 'Color', col, 'LineWidth', 2, ...
                 'DisplayName', r.label);
        if side == 1, legend_h(i) = h; end
        plot(ax, vk, fk, 'd', 'Color', col, 'MarkerSize', 8, ...
             'MarkerFaceColor', col, 'MarkerEdgeColor','k', 'LineWidth', 1, ...
             'HandleVisibility','off');
        plot(ax, [vk vk], [0 fk], '--', 'Color', [col 0.45], ...
             'LineWidth', 0.8, 'HandleVisibility','off');
    end
    xlim(ax,[0 inf]); ylim(ax,[0 inf]);
    xlabel(ax,'Absolute Velocity (mm/s)', 'FontSize',10, 'Color','k');
    ylabel(ax,'Force (N)', 'FontSize',10, 'Color','k');
    title(ax, sprintf('%s  %s — F vs |V| with Bi-linear Fit', WHEEL_PREFIX, curve_tags{side}), ...
          'FontSize',11, 'FontWeight','bold', 'Color','k');
    text(ax, 0.99, 0.03, char(9670) + " = knee / transition", 'Units','normalized', ...
         'HorizontalAlignment','right', 'FontSize',7.5, 'Color',[0.4 0.4 0.4]);
end
lgd = legend(ax_fv_c, legend_h, {res.label}, 'Location','northwest', 'FontSize',7.5, ...
             'TextColor','k', 'Color','w', 'EdgeColor',[0.4 0.4 0.4]);
lgd.Title.String   = 'Setting';
lgd.Title.FontSize = 8;
lgd.Title.Color    = 'k';

% ── Row 2: Bar charts ─────────────────────────────────────────────────────
ax_bar_c = subplot('Position', [0.05  0.07  0.42  0.40]);
ax_bar_r = subplot('Position', [0.55  0.07  0.42  0.40]);

draw_bar_panel(ax_bar_c, res, CMAP, 'comp', WHEEL_PREFIX);
draw_bar_panel(ax_bar_r, res, CMAP, 'reb',  WHEEL_PREFIX);

%% ── Figure 2: Summary table (dedicated window) ───────────────────────────
fig_tbl = figure('Name', sprintf('%s Damping Coefficient Table', WHEEL_PREFIX), ...
                 'Color','w', 'Position', [60 60 1500 200 + n_runs*100]);
ax_tbl = axes('Parent', fig_tbl, 'Position', [0.01 0.02 0.98 0.90]);
draw_summary_table(ax_tbl, res, CMAP, WHEEL_PREFIX);


%% ════════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
%% ════════════════════════════════════════════════════════════════════════

function [cv, cf, rv, rf] = extract_fv_curve(vel, force, n_bins, vel_thresh, pct)
    abs_vel = abs(vel);
    vel_max = prctile(abs_vel(abs_vel > vel_thresh), 99.5);
    edges   = linspace(vel_thresh, vel_max, n_bins + 1);
    bin_ctr = 0.5 * (edges(1:end-1) + edges(2:end));

    pos_vel_mask = vel >  vel_thresh;
    neg_vel_mask = vel < -vel_thresh;

    mean_f_pos = mean(force(pos_vel_mask));
    mean_f_neg = mean(force(neg_vel_mask));

    fprintf('    [sign detect]  vel>0 → mean force = %+.1f N  |  vel<0 → mean force = %+.1f N\n', ...
            mean_f_pos, mean_f_neg);

    if mean_f_pos >= 0
        comp_mask = pos_vel_mask;
        reb_mask  = neg_vel_mask;
        fprintf('    [sign detect]  Convention: vel>0 = COMPRESSION (+F),  vel<0 = REBOUND (-F)\n');
    else
        comp_mask = neg_vel_mask;
        reb_mask  = pos_vel_mask;
        fprintf('    [sign detect]  Convention: vel<0 = COMPRESSION (+F),  vel>0 = REBOUND (-F)\n');
    end

    cv = []; cf = []; rv = []; rf = [];

    for b = 1:n_bins
        in_bin = abs_vel >= edges(b) & abs_vel < edges(b+1);

        c_idx = comp_mask & in_bin;
        if sum(c_idx) >= 3
            fc  = force(c_idx);
            thr = prctile(fc, pct);
            sel = fc(fc >= thr);
            if ~isempty(sel)
                cv(end+1) = bin_ctr(b);    %#ok<AGROW>
                cf(end+1) = mean(sel);     %#ok<AGROW>
            end
        end

        r_idx = reb_mask & in_bin;
        if sum(r_idx) >= 3
            fr  = force(r_idx);
            thr = prctile(fr, 100 - pct);
            sel = fr(fr <= thr);
            if ~isempty(sel)
                rv(end+1) = bin_ctr(b);        %#ok<AGROW>
                rf(end+1) = abs(mean(sel));     %#ok<AGROW>
            end
        end
    end
end


function [c_ls, c_hs, v_knee, f_knee, r2] = fit_bilinear(v, f)
    n    = numel(v);
    best = struct('sse', inf, 'c_ls',0,'c_hs',0,'v_knee',0,'f_knee',0);

    for k = 4 : n-4
        v_k   = v(k);
        v_lo  = v(1:k);   f_lo  = f(1:k);
        c_ls_ = (v_lo(:)'*f_lo(:)) / (v_lo(:)'*v_lo(:));
        f_k   = c_ls_ * v_k;

        v_hi  = v(k:end); f_hi  = f(k:end);
        dv    = v_hi - v_k;
        df    = f_hi - f_k;
        den   = dv(:)'*dv(:);
        if den < 1e-9; continue; end
        c_hs_ = (dv(:)'*df(:)) / den;

        sse = sum((f_lo - c_ls_*v_lo).^2) + ...
              sum((f_hi - f_k - c_hs_*(v_hi - v_k)).^2);

        if sse < best.sse
            best.sse    = sse;
            best.c_ls   = c_ls_;
            best.c_hs   = c_hs_;
            best.v_knee = v_k;
            best.f_knee = f_k;
        end
    end

    c_ls   = best.c_ls;
    c_hs   = best.c_hs;
    v_knee = best.v_knee;
    f_knee = best.f_knee;

    f_pred = bilinear_eval(v, c_ls, c_hs, v_knee, f_knee);
    ss_res = sum((f - f_pred).^2);
    ss_tot = sum((f - mean(f)).^2);
    r2     = 1 - ss_res/max(ss_tot, eps);
end


function f = bilinear_eval(v, c_ls, c_hs, v_knee, f_knee)
    f = zeros(size(v));
    lo = v <= v_knee;
    f( lo) = c_ls * v(lo);
    f(~lo) = f_knee + c_hs .* (v(~lo) - v_knee);
end


function hatch_bar(ax, x_ctr, height, width, col)
    if height <= 0; return; end
    x0 = x_ctr - width/2;
    x1 = x_ctr + width/2;
    n_lines = 6;
    spacing = height / n_lines;
    for k = 1:n_lines
        y_bot = (k-1)*spacing;
        y_top = k*spacing;
        line(ax, [x0 x1], [y_bot y_top], 'Color', [col 0.55], ...
             'LineWidth', 0.9, 'HandleVisibility','off');
    end
end


function draw_summary_table(ax, res, CMAP, wheel_prefix)
    n_runs = numel(res);

    col_defs = {
        'Setting / HSc / HSr / LSc / LSr', 0.175, 'setting', '%s';
        'COMP C_ls (N.s/mm)',              0.103, 'c_ls_c',  '%.3f';
        'COMP V_knee (mm/s)',              0.103, 'vk_c',    '%.1f';
        'COMP C_hs (N.s/mm)',              0.103, 'c_hs_c',  '%.3f';
        'COMP R2',                         0.103, 'r2_c',    '%.4f';
        'REB C_ls (N.s/mm)',               0.103, 'c_ls_r',  '%.3f';
        'REB V_knee (mm/s)',               0.103, 'vk_r',    '%.1f';
        'REB C_hs (N.s/mm)',               0.103, 'c_hs_r',  '%.3f';
        'REB R2',                          0.103, 'r2_r',    '%.4f';
    };
    n_cols   = size(col_defs,1);
    c_widths = cell2mat(col_defs(:,2))';

    HDR_DARK  = [0.173 0.243 0.314];
    HDR_COMP  = [0.239 0.337 0.431];
    HDR_REB   = [0.180 0.251 0.341];
    HDR_KNEE  = [0.490 0.416 0.031];
    KNEE_BG   = [1.000 0.953 0.804];
    ALT_BG    = [0.941 0.961 0.973];
    WHITE_BG  = [1 1 1];

    BANNER_H = 0.07;
    HDR_H    = 0.20;
    ROW_H    = (0.88 - BANNER_H - HDR_H) / n_runs;

    axis(ax,'off'); hold(ax,'on');
    ax.XLim = [0 1]; ax.YLim = [0 1];

    text(ax, 0.5, 1.04, ...
         sprintf('%s  Damper Dyno -- Bi-linear Damping Coefficient Summary', wheel_prefix), ...
         'HorizontalAlignment','center','VerticalAlignment','bottom', ...
         'FontSize',20,'FontWeight','bold','Color',[0.1 0.1 0.1], ...
         'Interpreter','none','Clipping','off');

    comp_x = c_widths(1);  comp_w = sum(c_widths(2:5));
    reb_x  = comp_x+comp_w; reb_w = sum(c_widths(6:end));
    banner_y = 1.0 - BANNER_H;

    banner_info = {comp_x, comp_w, 'COMPRESSION', HDR_COMP;
                   reb_x,  reb_w,  'REBOUND',     HDR_REB};
    for bb = 1:2
        bx = banner_info{bb,1}; bw = banner_info{bb,2};
        bl = banner_info{bb,3}; bg = banner_info{bb,4};
        rectangle('Parent',ax,'Position',[bx banner_y bw BANNER_H], ...
                  'FaceColor',bg,'EdgeColor','none');
        text(ax, bx+bw/2, banner_y+BANNER_H/2, bl, ...
             'HorizontalAlignment','center','VerticalAlignment','middle', ...
             'FontSize',17,'FontWeight','bold','Color','w', ...
             'Interpreter','none','Clipping','off');
    end

    hdr_labels = {
        sprintf('Setting\n(%s / HSc/HSr/LSc/LSr)', res(1).wheel_code);
        sprintf('COMP\nC_ls (N.s/mm)');
        sprintf('COMP\nV_knee (mm/s)');
        sprintf('COMP\nC_hs (N.s/mm)');
        sprintf('COMP\nR^2');
        sprintf('REB\nC_ls (N.s/mm)');
        sprintf('REB\nV_knee (mm/s)');
        sprintf('REB\nC_hs (N.s/mm)');
        sprintf('REB\nR^2');
    };
    y_hdr = banner_y - HDR_H;
    x_cur = 0;
    for ci = 1:n_cols
        cw = c_widths(ci);
        if     ci == 1;             bg = HDR_DARK;
        elseif ci == 3 || ci == 7;  bg = HDR_KNEE;
        elseif ci <= 5;             bg = HDR_COMP;
        else;                       bg = HDR_REB;
        end
        lw = 1.5; ec = [0.15 0.15 0.15];
        rectangle('Parent',ax,'Position',[x_cur y_hdr cw HDR_H], ...
                  'FaceColor',bg,'EdgeColor',ec,'LineWidth',lw);
        text(ax, x_cur+cw/2, y_hdr+HDR_H/2, hdr_labels{ci}, ...
             'HorizontalAlignment','center','VerticalAlignment','middle', ...
             'FontSize',14,'FontWeight','bold','Color','w', ...
             'Interpreter','none','Clipping','off');
        x_cur = x_cur + cw;
    end

    for ri = 1:n_runs
        r     = res(ri);
        y_row = y_hdr - ri*ROW_H;
        alt   = mod(ri,2) == 1;

        cell_texts = {
            sprintf('%s  HSc%d / HSr%d\nLSc%d / LSr%d', ...
                    r.wheel_code, r.hs_c, r.hs_r, r.ls_c, r.ls_r);
            sprintf('%.3f', r.c_ls_c);
            sprintf('%.1f', r.vk_c);
            sprintf('%.3f', r.c_hs_c);
            sprintf('%.4f', r.r2_c);
            sprintf('%.3f', r.c_ls_r);
            sprintf('%.1f', r.vk_r);
            sprintf('%.3f', r.c_hs_r);
            sprintf('%.4f', r.r2_r);
        };

        x_cur = 0;
        for ci = 1:n_cols
            cw = c_widths(ci);

            if ci == 1
                bg = CMAP(ri,:); fg = [1 1 1]; fw = 'bold';
            elseif ci == 3 || ci == 7
                bg = KNEE_BG; fg = [0.1 0.1 0.1]; fw = 'normal';
            else
                if alt; bg = ALT_BG; else; bg = WHITE_BG; end
                fg = [0.1 0.1 0.1]; fw = 'normal';
            end

            if ci == 6; ec = [0.1 0.1 0.1]; lw = 1.5;
            else;        ec = [0.78 0.78 0.78]; lw = 0.4;
            end

            rectangle('Parent',ax,'Position',[x_cur y_row cw ROW_H], ...
                      'FaceColor',bg,'EdgeColor',ec,'LineWidth',lw);
            text(ax, x_cur+cw/2, y_row+ROW_H/2, cell_texts{ci}, ...
                 'HorizontalAlignment','center','VerticalAlignment','middle', ...
                 'FontSize',15,'FontWeight',fw,'Color',fg, ...
                 'Interpreter','none','Clipping','off');
            x_cur = x_cur + cw;
        end
    end

    foot_y = y_hdr - n_runs*ROW_H - 0.01;
    text(ax, 0.5, foot_y, ...
         ['C_ls = low-speed damping coeff.  |  V_knee = transition velocity  |  ' ...
          'C_hs = high-speed damping coeff.  |  R^2 = bi-linear fit quality'], ...
         'HorizontalAlignment','center','VerticalAlignment','top', ...
         'FontSize',13,'Color',[0.4 0.4 0.4],'FontAngle','italic', ...
         'Interpreter','none','Clipping','off');

    hold(ax,'off');
end


function draw_bar_panel(ax, res, CMAP, side, wheel_prefix)
    is_comp = strcmp(side,'comp');

    if is_comp
        ls_vals = [res.c_ls_c]' * 1000;
        hs_vals = [res.c_hs_c]' * 1000;
        vk_vals = [res.vk_c]';
        ttl     = sprintf('%s  Compression — Damping Coefficients', wheel_prefix);
    else
        ls_vals = [res.c_ls_r]' * 1000;
        hs_vals = [res.c_hs_r]' * 1000;
        vk_vals = [res.vk_r]';
        ttl     = sprintf('%s  Rebound — Damping Coefficients', wheel_prefix);
    end

    n_runs = numel(res);

    hold(ax,'on');
    ax.Color         = 'w';
    ax.FontSize      = 11;
    ax.Box           = 'on';
    ax.GridLineStyle = '--';
    ax.GridAlpha     = 0.25;
    ax.GridColor     = [0.55 0.55 0.55];
    ax.XColor        = 'k';
    ax.YColor        = 'k';

    W     = 0.30;
    GAP   = 0.10;
    x_ctr = (1:n_runs)';

    y_max = max([ls_vals; hs_vals]) * 1.28;
    if isnan(y_max) || y_max <= 0
        warning('plot_bar_figure: all damping values are zero or NaN for side=''%s''.', side);
        y_max = 1;
    end

    for i = 1:n_runs
        col  = CMAP(i,:);
        x_ls = x_ctr(i) - (W/2 + GAP/2);
        x_hs = x_ctr(i) + (W/2 + GAP/2);

        bar(ax, x_ls, ls_vals(i), W, ...
            'FaceColor', col, 'EdgeColor', 'none', ...
            'FaceAlpha', 1.0, 'LineWidth', 1.2, ...
            'HandleVisibility','off');

        bar(ax, x_hs, hs_vals(i), W, ...
            'FaceColor', col, 'EdgeColor', col, ...
            'FaceAlpha', 0.30, 'LineWidth', 1.0, ...
            'HandleVisibility','off');
        hatch_bar(ax, x_hs, hs_vals(i), W, col);

        pad = y_max * 0.025;
        text(ax, x_ls, ls_vals(i) + pad, sprintf('%.0f', ls_vals(i)), ...
             'HorizontalAlignment','center', 'VerticalAlignment','bottom', ...
             'FontSize',12, 'FontWeight','bold', 'Color', [0.1 0.1 0.1]);
        text(ax, x_hs, hs_vals(i) + pad, sprintf('%.0f', hs_vals(i)), ...
             'HorizontalAlignment','center', 'VerticalAlignment','bottom', ...
             'FontSize',12, 'FontWeight','bold', 'Color', [0.1 0.1 0.1]);
    end

    grid(ax,'on');

    set(ax, 'XTick', x_ctr, 'XTickLabel', {}, 'TickLength', [0 0]);

    ax.XLim = [0.4  n_runs + 0.6];
    ax.YLim = [0    y_max];

    for i = 1:n_runs
        col_i = CMAP(i,:);
        text(ax, x_ctr(i), -y_max * 0.045, ...
             res(i).wheel_code, ...
             'HorizontalAlignment','center', 'VerticalAlignment','top', ...
             'FontSize',12, 'FontWeight','bold', 'Color', col_i, ...
             'Interpreter','none');
        text(ax, x_ctr(i), -y_max * 0.095, ...
             sprintf('HSc%d / HSr%d\nLSc%d / LSr%d', ...
                     res(i).hs_c, res(i).hs_r, res(i).ls_c, res(i).ls_r), ...
             'HorizontalAlignment','center', 'VerticalAlignment','top', ...
             'FontSize',9, 'FontWeight','normal', 'Color', [0.30 0.30 0.30], ...
             'Interpreter','none');
        text(ax, x_ctr(i), -y_max * 0.195, ...
             sprintf('V_{kn}: %.0f mm/s', vk_vals(i)), ...
             'HorizontalAlignment','center', 'VerticalAlignment','top', ...
             'FontSize',9, 'Color',[0.35 0.35 0.35], 'FontAngle','italic');
    end

    set(ax, 'XColor', 'k', 'YColor', 'k');

    p1 = patch(ax, nan, nan, [0.50 0.50 0.50], ...
               'FaceAlpha',1.0, 'EdgeColor','none', ...
               'DisplayName','C_{ls}   low-speed slope');
    p2 = patch(ax, nan, nan, [0.50 0.50 0.50], ...
               'FaceAlpha',0.30, 'EdgeColor',[0.50 0.50 0.50], ...
               'DisplayName','C_{hs}   high-speed slope');
    lgd = legend(ax, [p1 p2], 'Location','northeastoutside', 'FontSize',10, 'Box','on', ...
                 'TextColor','k', 'Color','w', 'EdgeColor',[0.4 0.4 0.4]);
    lgd.Title.String  = 'Coefficient';
    lgd.Title.Color   = 'k';

    xlabel(ax, 'Wheel  /  Setting  (HS comp  |  HS reb  |  LS comp  |  LS reb)', ...
           'FontSize',11, 'Color','k');
    ylabel(ax, 'Damping Coefficient  (Ns/m)', 'FontSize',12, 'Color','k');
    title(ax, ttl, 'FontSize',14, 'FontWeight','bold', 'Color','k');
end