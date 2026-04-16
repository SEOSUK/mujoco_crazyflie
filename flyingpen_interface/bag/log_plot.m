%% log_plot.m
% Reads data_logger CSV and builds a comparison dashboard.
% Legacy CSV (46 cols), MOB force CSV (49 cols), MOB vs second-order compare
% CSV (58 cols), MOB vs second-order vs consistency compare CSV (64 cols),
% and consistency-debug CSV (76 cols) are supported.

clear; clc;
set(groot, 'defaultFigureRenderer', 'painters');

%% ---- pick CSV ----
defaultDir = fullfile(getenv("HOME"), "mujoco_crazyflie", "src", "flyingpen_interface", "bag");
if ~isfolder(defaultDir)
    defaultDir = pwd;
end

[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select data_logger CSV");
if isequal(file, 0)
    disp("Canceled.");
    return;
end
csvPath = fullfile(path, file);

%% ---- read ----
opts = detectImportOptions(csvPath);
opts = setvartype(opts, 'double');
T = readtable(csvPath, opts);
A = table2array(T);

if size(A, 2) < 46
    error("CSV must have at least 46 columns, got %d", size(A, 2));
end

col = @(k) A(:, k);
time = col(1);
time = time - time(1);

%% ---- parse base fields ----
cmd_xyzyaw = [col(2), col(3), col(4), col(5)];
pose_xyz = [col(6), col(7), col(8)];
att_pose = [col(9), col(10), unwrap(col(11))];
vel_xyz = [col(12), col(13), col(14)];
w_xyz = [col(15), col(16), col(17)];
acc_xyz = [col(18), col(19), col(20)];
angacc_xyz = [col(21), col(22), col(23)];
vdes_xyz = [col(24), col(25), col(26)];
att_des_u = [col(27), col(28), unwrap(col(29))];
wdes_xyz = [col(30), col(31), col(32)];
tau_xyz = [col(33), col(34), col(35)];
Fz = col(36);
contact_raw = [col(37), col(38), col(39)];
contact_filt = [col(40), col(41), col(42)];
cmd_force = col(43);
lpf_force_raw = col(44);
lpf_force_filt = col(45);
mask = uint32(col(46));

if size(A, 2) >= 49
    mob_force = [col(47), col(48), col(49)];
    mob_force_valid = true;
else
    mob_force = zeros(size(A, 1), 3);
    mob_force_valid = false;
    warning('MOB force columns not found. Using zeros. Re-log with updated data_logger for panel (2,2).');
end

if size(A, 2) >= 58
    mob1_force = [col(47), col(48), col(49)];
    mob1_torque = [col(50), col(51), col(52)];
    mob2_force = [col(53), col(54), col(55)];
    mob2_torque = [col(56), col(57), col(58)];
    mob_compare_valid = true;
else
    z3 = zeros(size(A,1), 3);
    mob1_force = z3; mob1_torque = z3;
    mob2_force = z3; mob2_torque = z3;
    mob_compare_valid = false;
    warning('MOB and second-order force/torque columns not found. Re-log with updated data_logger for 3x2 MOB compare figure.');
end

if size(A, 2) >= 64
    mob3_force = [col(59), col(60), col(61)];
    mob3_torque = [col(62), col(63), col(64)];
    mob_compare_3way_valid = true;
else
    z3 = zeros(size(A,1), 3);
    mob3_force = z3; mob3_torque = z3;
    mob_compare_3way_valid = false;
end

if size(A, 2) >= 76
    mob_tau_kfep = [col(65), col(66), col(67)];
    mob_tau_consistency = [col(68), col(69), col(70)];
    mob_tau_tauhat = [col(71), col(72), col(73)];
    mob_tau_rxf = [col(74), col(75), col(76)];
    mob_tau_debug_valid = true;
else
    z3 = zeros(size(A,1), 3);
    mob_tau_kfep = z3;
    mob_tau_consistency = z3;
    mob_tau_tauhat = z3;
    mob_tau_rxf = z3;
    mob_tau_debug_valid = false;
end

%% ---- comparison signals ----
mass_nominal = 0.04338;
mass_acc = mass_nominal .* acc_xyz;
force_world_default_cmp = thrust_world_from_pose_and_fz(att_pose(:,1), att_pose(:,2), att_pose(:,3), Fz);
force_world_default_cmp(:,3) = force_world_default_cmp(:,3) - mass_nominal * 9.81;

axis_names = {'X', 'Y', 'Z'};
att_names = {'roll', 'pitch', 'yaw'};

%% Figure 5) 2x2 dashboard with 3x1 subplots per panel
twin = [0 20];
xlim_cfg_fig5_pos = twin;
xlim_cfg_fig5_ma  = twin;
xlim_cfg_fig5_att = twin;
xlim_cfg_fig5_mob = twin;

ylim_cfg_fig5_pos = [-0.5 0.7; -0.6 0.8; 0.0 1.2];
ylim_cfg_fig5_ma  = [-0.1 0.1; -0.1 0.1; -0.1 0.1];
ylim_cfg_fig5_att = [-0.3 0.3; -0.3 0.3; -0.3 0.3];
ylim_cfg_fig5_mob = [-0.05 0.05; -0.05 0.05; -0.05 0.05];

f5 = figure('Name','Tracking / Force dashboard','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.04 0.05 0.92 0.88]);

if mob_force_valid
    mob_status_text = sprintf('MOB source: logged /crazyflie/out/mob (%s)', file);
    mob_status_color = [0.10 0.45 0.10];
else
    mob_status_text = sprintf('MOB source: NOT in CSV, panel (2,2) uses zeros (%s)', file);
    mob_status_color = [0.75 0.15 0.10];
end

annotation(f5, 'textbox', [0.04 0.955 0.92 0.03], ...
    'String', mob_status_text, ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'left', ...
    'Interpreter', 'none', ...
    'FontWeight', 'bold', ...
    'Color', mob_status_color);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 2; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position XYZ (des vs actual)
p11 = uipanel('Parent', f5, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_pos = gobjects(0);
for i = 1:3
    ax = nexttile(tl11, i); ax_fig5_pos(end+1) = ax; %#ok<SAGROW>
    plot(time, pose_xyz(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_pos, i, [cmd_xyzyaw(:,i); pose_xyz(:,i)]);
    ylabel(sprintf('%s [m]', axis_names{i}));
    if i == 1
        title('Position');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_pos, xlim_cfg_fig5_pos, 'data_decryptor_specific_fig5_pos_xlink');
xlabel(tl11, 'time [s]');

% (1,2) scaled force vs mass * acceleration
p12 = uipanel('Parent', f5, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_ma = gobjects(0);
for i = 1:3
    ax = nexttile(tl12, i); ax_fig5_ma(end+1) = ax; %#ok<SAGROW>
    plot(time, mass_acc(:,i), '-', 'LineWidth', 1.35, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_ma, i, [force_world_default_cmp(:,i); mass_acc(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. mass * acceleration');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_ma, xlim_cfg_fig5_ma, 'data_decryptor_specific_fig5_ma_xlink');
xlabel(tl12, 'time [s]');

% (2,1) attitude des vs actual
p21 = uipanel('Parent', f5, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl21 = tiledlayout(p21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_att = gobjects(0);
for i = 1:3
    ax = nexttile(tl21, i); ax_fig5_att(end+1) = ax; %#ok<SAGROW>
    plot(time, att_pose(:,i), '-', 'LineWidth', 1.15); hold on;
    plot(time, att_des_u(:,i), '--', 'LineWidth', 0.95, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_att, i, [att_des_u(:,i); att_pose(:,i)]);
    ylabel(sprintf('%s [rad]', att_names{i}));
    if i == 1
        title('Attitude');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_att, xlim_cfg_fig5_att, 'data_decryptor_specific_fig5_att_xlink');
xlabel(tl21, 'time [s]');

% (2,2) scaled force vs MOB force
p22 = uipanel('Parent', f5, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig5_mob = gobjects(0);
for i = 1:3
    ax = nexttile(tl22, i); ax_fig5_mob(end+1) = ax; %#ok<SAGROW>
    plot(time, mob_force(:,i), '-', 'LineWidth', 2.4, 'Color', [0.25 0.55 0.95]); hold on;
    plot(time, force_world_default_cmp(:,i), 'LineWidth', 1.05);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig5_mob, i, [force_world_default_cmp(:,i); mob_force(:,i)]);
    ylabel(sprintf('%s [N]', axis_names{i}));
    if i == 1
        title('F_{thrust} vs. Momentum Observer');
    end
    if i < 3
        ax.XTickLabel = [];
    end
end
apply_panel_xlim(ax_fig5_mob, xlim_cfg_fig5_mob, 'data_decryptor_specific_fig5_mob_xlink');
xlabel(tl22, 'time [s]');

set(findall(f5, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

disp("Done. Variables prepared:");
disp("- pose_xyz / cmd_xyzyaw");
disp("- mass_acc / force_world_default_cmp");
disp("- att_pose / att_des_u");
disp("- mob_force");
if mob_force_valid
    disp("- MOB source: logged /crazyflie/out/mob");
else
    disp("- MOB source: missing in this CSV, using zeros in panel (2,2)");
end
if mob_compare_3way_valid
    disp("- MOB compare source: logged /crazyflie/out/mob, /crazyflie/out/mob_2nd, /crazyflie/out/mob_2nd_tau force/torque");
elseif mob_compare_valid
    disp("- MOB compare source: logged /crazyflie/out/mob and /crazyflie/out/mob_2nd force/torque");
else
    disp("- MOB compare source: missing in this CSV, using zeros in 3x2 compare figure");
end
if mob_tau_debug_valid
    disp("- MOB consistency debug source: logged /crazyflie/out/mob_2nd_tau_terms and /crazyflie/out/mob_2nd_tau_consistency");
else
    disp("- MOB consistency debug source: missing in this CSV, using zeros in Figure 6 right column");
end

%% Figure 6) MOB 2nd-order only + consistency observer diagnostics
xlim_cfg_fig6 = twin;
ylim_cfg_fig6_force = [-0.01 0.1; -0.05 0.05; -0.05 0.05];

mob_tau_kfep_norm = vecnorm(mob_tau_kfep, 2, 2);
mob_tau_consistency_norm = vecnorm(mob_tau_consistency, 2, 2);
mob_tau_tauhat_norm = vecnorm(mob_tau_tauhat, 2, 2);
mob_tau_rxf_norm = vecnorm(mob_tau_rxf, 2, 2);
mob_tau_residual_norm = vecnorm(mob_tau_tauhat - mob_tau_rxf, 2, 2);

f6 = figure('Name','MOB 2nd Order and Consistency Diagnostics','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.08 0.08 0.84 0.82]);
tl6 = tiledlayout(f6, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_fig6 = gobjects(0);

for i = 1:3
    ax = nexttile(tl6, 2*(i-1)+1); ax_fig6(end+1) = ax; %#ok<SAGROW>
    plot(time, mob2_force(:,i), 'LineWidth', 1.2, 'Color', [0.20 0.50 0.95]); hold on;
    plot(time, contact_filt(:,i), '--', 'LineWidth', 1.2, 'Color', [0.10 0.10 0.10]);
    grid on;
    apply_user_ylim(ax, ylim_cfg_fig6_force, i, [mob2_force(:,i); contact_filt(:,i)]);
    ylabel(sprintf('F%s [N]', lower(axis_names{i})));
    if i == 1
        title('External Force');
        legend({'2nd order', 'contact'}, 'Location', 'best');
    end
    if i < 3
        ax.XTickLabel = [];
    else
        xlabel('time [s]');
    end

    ax = nexttile(tl6, 2*(i-1)+2); ax_fig6(end+1) = ax; %#ok<SAGROW>
    if i == 1
        plot(time, mob_tau_kfep_norm, 'LineWidth', 1.3, 'Color', [0.15 0.45 0.90]); hold on;
        plot(time, mob_tau_consistency_norm, 'LineWidth', 1.3, 'Color', [0.90 0.35 0.15]);
        ylabel('norm [N/s]');
        title('Force Update Contribution');
        legend({'baseline term: ||K_f e_p||', 'consistency term: ||\sigma_\tau K_e [r]^T_\times e_\tau||'}, 'Location', 'best');
        apply_user_ylim(ax, [nan nan; nan nan; nan nan], 1, [mob_tau_kfep_norm; mob_tau_consistency_norm]);
    elseif i == 2
        plot(time, mob_tau_tauhat_norm, 'LineWidth', 1.3, 'Color', [0.20 0.65 0.35]); hold on;
        plot(time, mob_tau_rxf_norm, 'LineWidth', 1.3, 'Color', [0.55 0.25 0.80]);
        ylabel('norm [Nm]');
        title('Torque Consistency Match');
        legend({'estimated torque: ||\hat{\tau}^w_{ext}||', 'contact-induced moment: ||r^w \times \hat{f}^w_{ext}||'}, 'Location', 'best');
        apply_user_ylim(ax, [nan nan; nan nan; nan nan], 1, [mob_tau_tauhat_norm; mob_tau_rxf_norm]);
    else
        plot(time, mob_tau_residual_norm, 'LineWidth', 1.3, 'Color', [0.10 0.10 0.10]); hold on;
        ylabel('norm [Nm]');
        title('Residual After Correction');
        legend({'consistency residual: ||e_\tau||'}, 'Location', 'best');
        apply_user_ylim(ax, [nan nan; nan nan; nan nan], 1, mob_tau_residual_norm);
    end
    grid on;
    if i < 3
        ax.XTickLabel = [];
    else
        xlabel('time [s]');
    end
end

apply_panel_xlim(ax_fig6, xlim_cfg_fig6, 'data_decryptor_specific_fig6_mob_compare_xlink');
set(findall(f6, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

function force_world = thrust_world_from_pose_and_fz(roll, pitch, yaw, fz)
    n = numel(fz);
    force_world = zeros(n, 3);
    for k = 1:n
        cr = cos(roll(k)); sr = sin(roll(k));
        cp = cos(pitch(k)); sp = sin(pitch(k));
        cy = cos(yaw(k)); sy = sin(yaw(k));
        r_bw = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
                sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
                -sp,   cp*sr,            cp*cr];
        force_world(k, :) = (r_bw * [0; 0; fz(k)]).';
    end
end

function apply_user_ylim(ax, ylim_cfg, idx, fallback_data)
    if size(ylim_cfg, 1) < idx
        return;
    end
    yv = ylim_cfg(idx, :);
    if all(isfinite(yv))
        ylim(ax, yv);
        return;
    end

    data = fallback_data(isfinite(fallback_data));
    if isempty(data)
        return;
    end
    dmin = min(data);
    dmax = max(data);
    if dmin == dmax
        pad = max(1e-3, 0.05 * max(1.0, abs(dmin)));
    else
        pad = 0.08 * (dmax - dmin);
    end
    ylim(ax, [dmin - pad, dmax + pad]);
end

function apply_panel_xlim(axs, xlim_cfg, link_tag)
    if ~isempty(link_tag)
        linkaxes(axs, 'x');
    end
    if numel(xlim_cfg) == 2 && all(isfinite(xlim_cfg))
        for i = 1:numel(axs)
            xlim(axs(i), xlim_cfg);
        end
    end
end