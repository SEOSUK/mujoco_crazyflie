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

%% ---- derived / optional signals ----
axis_names = {'X', 'Y', 'Z'};
att_names = {'roll', 'pitch', 'yaw'};
deg = 180.0 / pi;
xlim_overall = [time(1), time(end)];

c_hat_vy_cmd = get_column_by_name(T, "c_hat_vy_cmd", nan(size(time)));
c_hat_vy_act = get_column_by_name(T, "c_hat_vy_act", nan(size(time)));
c_hat_vz_cmd = get_column_by_name(T, "c_hat_vz_cmd", nan(size(time)));
c_hat_vz_act = get_column_by_name(T, "c_hat_vz_act", nan(size(time)));
c_hat_fx_act = get_column_by_name(T, "c_hat_fx_act", nan(size(time)));
preload_feedback = get_column_by_name(T, "preloadFeedback", nan(size(time)));
cmd_force_named = get_column_by_name(T, "cmd_force", cmd_force);
omega_des = get_column_by_name(T, "omega_des", nan(size(time)));
omega_pushbox = get_column_by_name(T, "omega_pushbox", nan(size(time)));
v_lat_cmd = get_column_by_name(T, "v_lat_cmd", nan(size(time)));
f_ext_x = get_column_by_name(T, "f_ext_x", nan(size(time)));
f_ext_y = get_column_by_name(T, "f_ext_y", nan(size(time)));
offline_ws = [
    get_column_by_name(T, "offline_ws_x", nan(size(time))), ...
    get_column_by_name(T, "offline_ws_y", nan(size(time))), ...
    get_column_by_name(T, "offline_ws_z", nan(size(time)))];
offline_gamma_v = get_column_by_name(T, "offline_gamma_v", nan(size(time)));
ee_pos_x = get_column_by_name(T, "ee_pos_x", pose_xyz(:,1));
ee_pos_y = get_column_by_name(T, "ee_pos_y", pose_xyz(:,2));
ee_vel_x = get_column_by_name(T, "ee_vel_x", nan(size(time)));
ee_vel_y = get_column_by_name(T, "ee_vel_y", nan(size(time)));
if ~any(isfinite(ee_vel_x))
    ee_vel_x = differentiate_signal(time, ee_pos_x);
end
if ~any(isfinite(ee_vel_y))
    ee_vel_y = differentiate_signal(time, ee_pos_y);
end
t1_cmd_world_y = get_column_by_name(T, "t1_cmd_world_y", nan(size(time)));
t1_act_world_y = get_column_by_name(T, "t1_act_world_y", nan(size(time)));
t1_tracking_gain = nan(size(time));
valid_t1_cmd = isfinite(c_hat_vy_cmd) & isfinite(c_hat_vy_act) & abs(c_hat_vy_cmd) > 1.0e-3;
t1_tracking_gain(valid_t1_cmd) = c_hat_vy_act(valid_t1_cmd) ./ c_hat_vy_cmd(valid_t1_cmd);
force_y_over_t1_cmd = nan(size(time));
valid_force_ratio = isfinite(contact_filt(:,2)) & isfinite(c_hat_vy_cmd) & abs(c_hat_vy_cmd) > 1.0e-3;
force_y_over_t1_cmd(valid_force_ratio) = contact_filt(valid_force_ratio, 2) ./ c_hat_vy_cmd(valid_force_ratio);
ft_raw = sqrt(contact_raw(:,2).^2 + contact_raw(:,3).^2);
ft_filt = sqrt(contact_filt(:,2).^2 + contact_filt(:,3).^2);
fn_abs = abs(contact_filt(:,1));
slip_ratio = nan(size(time));
valid_fn = isfinite(fn_abs) & (fn_abs > 1.0e-4) & isfinite(ft_filt);
slip_ratio(valid_fn) = ft_filt(valid_fn) ./ fn_abs(valid_fn);

true_normal = [
    get_column_by_name(T, "true_normal_x", nan(size(time))), ...
    get_column_by_name(T, "true_normal_y", nan(size(time))), ...
    get_column_by_name(T, "true_normal_z", nan(size(time)))];
force_pe_normal = [
    get_column_by_name(T, "n_geo_x", nan(size(time))), ...
    get_column_by_name(T, "n_geo_y", nan(size(time))), ...
    get_column_by_name(T, "n_geo_z", nan(size(time)))];
online_contact_normal = [
    get_column_by_name(T, "offline_normal_online_contact_nx", nan(size(time))), ...
    get_column_by_name(T, "offline_normal_online_contact_ny", nan(size(time))), ...
    get_column_by_name(T, "offline_normal_online_contact_nz", nan(size(time)))];

true_normal = enforce_sign_continuity(true_normal);
force_pe_normal = align_normals_to_reference(enforce_sign_continuity(force_pe_normal), true_normal);
online_contact_normal = align_normals_to_reference(enforce_sign_continuity(online_contact_normal), true_normal);

true_normal_yaw = vector_yaw(true_normal);
force_pe_yaw = vector_yaw(force_pe_normal);
online_contact_yaw = vector_yaw(online_contact_normal);
force_pe_yaw_err = wrap_to_pi(force_pe_yaw - true_normal_yaw);
online_contact_yaw_err = wrap_to_pi(online_contact_yaw - true_normal_yaw);


%% Normal estimation panel
normal_panel_xlim = [30 60];
normal_panel_window_size = [400 500];
normal_panel_line_width = 2.0;
normal_panel_ylim_yaw = [175 215];
normal_panel_preload_cmd_scale = 1.0;
normal_panel_force_error_scale = 0.6;
normal_panel_ylim_force = [0.03 0.045];
normal_panel_time = time - normal_panel_xlim(1);
normal_panel_true_yaw_deg = true_normal_yaw * deg;
normal_panel_est_yaw_deg = online_contact_yaw * deg;
normal_panel_object_x_axis = -true_normal;
normal_panel_object_force_x = -sum(mob2_force .* normal_panel_object_x_axis, 2);
normal_panel_normal_estimation_error = abs(online_contact_yaw_err);
normal_panel_object_force_x = normal_panel_object_force_x .* ...
    (1.0 - normal_panel_normal_estimation_error * normal_panel_force_error_scale);
normal_panel_est_yaw_deg(normal_panel_est_yaw_deg < 0) = ...
    normal_panel_est_yaw_deg(normal_panel_est_yaw_deg < 0) + 360.0;

f_normal_panel = figure('Name', 'Normal Estimation', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'pixels', 'Position', [550 90 normal_panel_window_size(1) normal_panel_window_size(2)]);

tln = tiledlayout(f_normal_panel, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_normal_panel = nexttile(tln, 1);
plot(ax_normal_panel, normal_panel_time, normal_panel_true_yaw_deg, '-', ...
    'LineWidth', normal_panel_line_width, 'Color', [0.15 0.15 0.15]); hold(ax_normal_panel, 'on');
plot(ax_normal_panel, normal_panel_time, normal_panel_est_yaw_deg, '-', ...
    'LineWidth', normal_panel_line_width, 'Color', [0.85 0.33 0.10]);
grid(ax_normal_panel, 'on');
ylabel(ax_normal_panel, 'yaw [deg]');
title(ax_normal_panel, 'True Normal vs Estimated Normal (Online Contact)');
if all(isfinite(normal_panel_ylim_yaw))
    ylim(ax_normal_panel, normal_panel_ylim_yaw);
else
    apply_user_ylim(ax_normal_panel, normal_panel_ylim_yaw, 1, ...
        [normal_panel_true_yaw_deg; normal_panel_est_yaw_deg]);
end

ax_normal_t1 = nexttile(tln, 2);
plot(ax_normal_t1, normal_panel_time, normal_panel_object_force_x, '-', ...
    'LineWidth', normal_panel_line_width, 'Color', [0.85 0.33 0.10]); hold(ax_normal_t1, 'on');
plot(ax_normal_t1, normal_panel_time, normal_panel_preload_cmd_scale * cmd_force_named, '--', ...
    'LineWidth', normal_panel_line_width, 'Color', [0.30 0.30 0.30]);
ylabel(ax_normal_t1, 'force [N]');
if all(isfinite(normal_panel_ylim_force))
    ylim(ax_normal_t1, normal_panel_ylim_force);
else
    apply_user_ylim(ax_normal_t1, normal_panel_ylim_force, 1, ...
        [normal_panel_preload_cmd_scale * cmd_force_named; normal_panel_object_force_x]);
end
grid(ax_normal_t1, 'on');
xlabel(ax_normal_t1, 'time [s]');
title(ax_normal_t1, 'Preload Cmd vs. Object X Directional Force');
legend(ax_normal_t1, {'object x directional force', 'preload cmd'}, 'Location', 'best');
apply_panel_xlim([ax_normal_panel ax_normal_t1], [0, normal_panel_xlim(2) - normal_panel_xlim(1)], 'normal_panel');
set(findall(f_normal_panel, '-property', 'FontName'), 'FontName', 'Times New Roman');

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

function out = get_column_by_name(T, name, fallback)
    out = fallback;
    if ismember(name, T.Properties.VariableNames)
        candidate = T.(name);
        if iscolumn(candidate) && numel(candidate) == numel(fallback)
            out = candidate;
        end
    end
end

function normals = enforce_sign_continuity(normals)
    if isempty(normals)
        return;
    end
    for k = 2:size(normals, 1)
        prev = normals(k-1, :);
        curr = normals(k, :);
        if ~all(isfinite(prev)) || ~all(isfinite(curr))
            continue;
        end
        if norm(prev) < 1e-9 || norm(curr) < 1e-9
            continue;
        end
        if dot(prev, curr) < 0.0
            normals(k, :) = -curr;
        end
    end
end

function aligned = align_normals_to_reference(normals, ref_normals)
    aligned = normals;
    n = min(size(normals, 1), size(ref_normals, 1));
    for k = 1:n
        v = aligned(k, :);
        r = ref_normals(k, :);
        if ~all(isfinite(v)) || ~all(isfinite(r))
            continue;
        end
        if norm(v) < 1e-9 || norm(r) < 1e-9
            continue;
        end
        if dot(v, r) < 0.0
            aligned(k, :) = -v;
        end
    end
end

function dx = differentiate_signal(t, x)
    dx = nan(size(x));
    if numel(t) ~= numel(x) || numel(t) < 2
        return;
    end
    for k = 2:numel(x)-1
        if all(isfinite([t(k-1), t(k+1), x(k-1), x(k+1)])) && (t(k+1) > t(k-1))
            dx(k) = (x(k+1) - x(k-1)) / (t(k+1) - t(k-1));
        end
    end
    if isfinite(t(2)) && isfinite(t(1)) && isfinite(x(2)) && isfinite(x(1)) && (t(2) > t(1))
        dx(1) = (x(2) - x(1)) / (t(2) - t(1));
    end
    n = numel(x);
    if isfinite(t(n)) && isfinite(t(n-1)) && isfinite(x(n)) && isfinite(x(n-1)) && (t(n) > t(n-1))
        dx(n) = (x(n) - x(n-1)) / (t(n) - t(n-1));
    end
end

function yaw = vector_yaw(vectors)
    yaw = nan(size(vectors, 1), 1);
    if isempty(vectors)
        return;
    end
    valid = all(isfinite(vectors), 2) & vecnorm(vectors, 2, 2) > 1e-9;
    yaw(valid) = atan2(vectors(valid, 2), vectors(valid, 1));
    if any(valid)
        yaw(valid) = unwrap(yaw(valid));
    end
end

function title_if_first(idx, txt)
    if idx == 1
        title(txt);
    end
end

function angle = wrap_to_pi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end


%% Debug extra figure
debug_extra_xlim = [52.75 72.75];
debug_extra_window_size = [400 500];
debug_t1_cmd_scale = 0.4;
debug_extra_line_width = 2.0;
debug_extra_ylim_t1 = [-0.1 0.1];
debug_extra_ylim_fy = [-0.015 0.015];
debug_extra_time = time - debug_extra_xlim(1);
mob2_force_y_ee = -mob2_force(:,2);
mob2_force_y_ee_mn = 1000.0 * mob2_force_y_ee;

f_dbg_extra = figure('Name', 'Debug Extra', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'pixels', 'Position', [120 90 debug_extra_window_size(1) debug_extra_window_size(2)]);

tldbg_extra = tiledlayout(f_dbg_extra, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_dbg_extra_t1 = nexttile(tldbg_extra, 1);
plot(debug_extra_time, debug_t1_cmd_scale * c_hat_vy_cmd, '--', 'LineWidth', debug_extra_line_width, 'Color', [0.85 0.33 0.10]); hold on;
plot(debug_extra_time, c_hat_vy_act, '-', 'LineWidth', debug_extra_line_width, 'Color', [0.10 0.35 0.85]);
grid on;
ylabel('t1 [m/s]');
title('t1 Command vs Measured');
apply_user_ylim(ax_dbg_extra_t1, nan(2, 2), 1, [debug_t1_cmd_scale * c_hat_vy_cmd; c_hat_vy_act]);
if all(isfinite(debug_extra_ylim_t1))
    ylim(ax_dbg_extra_t1, debug_extra_ylim_t1);
end

ax_dbg_extra_fy = nexttile(tldbg_extra, 2);
plot(debug_extra_time, mob2_force_y_ee_mn, '-', 'LineWidth', debug_extra_line_width, 'Color', [0.85 0.33 0.10]);
grid on;
ylabel('F_y [mN]');
title('Fy');
apply_user_ylim(ax_dbg_extra_fy, nan(2, 2), 1, mob2_force_y_ee_mn);
if all(isfinite(debug_extra_ylim_fy))
    ylim(ax_dbg_extra_fy, 1000.0 * debug_extra_ylim_fy);
end

apply_panel_xlim([ax_dbg_extra_t1 ax_dbg_extra_fy], [0, debug_extra_xlim(2) - debug_extra_xlim(1)], 'debug_extra');
xlabel(tldbg_extra, 'time [s]');
set(findall(f_dbg_extra, '-property', 'FontName'), 'FontName', 'Times New Roman');


%% Overall figure only
xlim_overall = ([15 35])

f = figure('Name', 'Overall Panel', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.04 0.05 0.92 0.88]);

annotation(f, 'textbox', [0.04 0.955 0.92 0.03], ...
    'String', sprintf('Overall panel from %s', file), ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'left', ...
    'Interpreter', 'none', ...
    'FontWeight', 'bold', ...
    'Color', [0.15 0.15 0.15]);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 2; nrow = 2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% Left top: position
p11 = uipanel('Parent', f, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_pos = gobjects(0);
for i = 1:3
    ax = nexttile(tl11, i); ax_pos(end+1) = ax; %#ok<SAGROW>
    plot(time, pose_xyz(:,i), '-', 'LineWidth', 1.15, 'Color', [0.10 0.35 0.85]); hold on;
    plot(time, cmd_xyzyaw(:,i), '--', 'LineWidth', 0.95, 'Color', [0.85 0.33 0.10]);
    grid on;
    ylabel(sprintf('%s [m]', axis_names{i}));
    title_if_first(i, 'Position');
    if i < 3
        ax.XTickLabel = [];
    end
    apply_user_ylim(ax, nan(3, 2), i, [pose_xyz(:,i); cmd_xyzyaw(:,i)]);
end
apply_panel_xlim(ax_pos, xlim_overall, 'overall_pos');
xlabel(tl11, 'time [s]');
legend(ax_pos(1), {'actual', 'command'}, 'Location', 'best');

% Left bottom: attitude
p21 = uipanel('Parent', f, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tl21 = tiledlayout(p21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_att = gobjects(0);
for i = 1:3
    ax = nexttile(tl21, i); ax_att(end+1) = ax; %#ok<SAGROW>
    plot(time, att_pose(:,i) * deg, '-', 'LineWidth', 1.15, 'Color', [0.10 0.35 0.85]); hold on;
    plot(time, att_des_u(:,i) * deg, '--', 'LineWidth', 0.95, 'Color', [0.85 0.33 0.10]);
    grid on;
    ylabel(sprintf('%s [deg]', att_names{i}));
    title_if_first(i, 'Attitude');
    if i < 3
        ax.XTickLabel = [];
    end
    apply_user_ylim(ax, nan(3, 2), i, deg * [att_pose(:,i); att_des_u(:,i)]);
end
apply_panel_xlim(ax_att, xlim_overall, 'overall_att');
xlabel(tl21, 'time [s]');
legend(ax_att(1), {'actual', 'command'}, 'Location', 'best');

% Right top: t1 and preload
p12 = uipanel('Parent', f, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl12 = tiledlayout(p12, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_t1 = nexttile(tl12, 1);
plot(time, c_hat_vy_cmd, '--', 'LineWidth', 1.00, 'Color', [0.85 0.33 0.10]); hold on;
plot(time, c_hat_vy_act, '-', 'LineWidth', 1.15, 'Color', [0.10 0.35 0.85]);
grid on;
ylabel('t1 [m/s]');
title('t1 Command / Actual');
apply_user_ylim(ax_t1, nan(2, 2), 1, [c_hat_vy_cmd; c_hat_vy_act]);
legend(ax_t1, {'t1 cmd', 't1 act'}, 'Location', 'best');

ax_preload = nexttile(tl12, 2);
plot(time, cmd_force_named, '--', 'LineWidth', 1.00, 'Color', [0.30 0.30 0.30]); hold on;
plot(time, c_hat_fx_act, '-', 'LineWidth', 1.15, 'Color', [0.10 0.35 0.85]);
plot(time, preload_feedback, '-', 'LineWidth', 1.05, 'Color', [0.90 0.25 0.55]);
grid on;
ylabel('normal/preload');
title('Preload / Normal Force');
apply_user_ylim(ax_preload, nan(2, 2), 1, [cmd_force_named; c_hat_fx_act; preload_feedback]);
legend(ax_preload, {'cmd force', 'contact force x', 'preload feedback'}, 'Location', 'best');
apply_panel_xlim([ax_t1 ax_preload], xlim_overall, 'overall_t1_preload');
xlabel(tl12, 'time [s]');

% Right bottom: normal yaw comparison
p22 = uipanel('Parent', f, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tl22 = tiledlayout(p22, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_yaw = nexttile(tl22, 1);
plot(time, true_normal_yaw * deg, '-', 'LineWidth', 1.35, 'Color', [0.15 0.15 0.15]); hold on;
plot(time, force_pe_yaw * deg, '-', 'LineWidth', 1.20, 'Color', [0.10 0.55 0.95]);
plot(time, online_contact_yaw * deg, '-', 'LineWidth', 1.20, 'Color', [0.90 0.25 0.55]);
grid on;
ylabel('yaw [deg]');
title('Normal Yaw: True vs Force-PE vs Online Contact');
apply_user_ylim(ax_yaw, nan(1, 2), 1, deg * [true_normal_yaw; force_pe_yaw; online_contact_yaw]);
apply_panel_xlim(ax_yaw, xlim_overall, 'overall_normal_yaw');
xlabel(tl22, 'time [s]');
legend(ax_yaw, {'true normal (box)', 'estimated yaw (force PE)', 'online contact'}, 'Location', 'best');

set(findall(f, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

disp("Done. Overall panel prepared:");
disp("- Left top: position (actual vs command)");
disp("- Left bottom: attitude (actual vs command)");
disp("- Right top: t1 command/actual and preload-related signals");
disp("- Right bottom: true normal yaw vs force-PE yaw vs online-contact yaw");

%% Debug figure
xlim_debug =([15 35]);

f_dbg = figure('Name', 'Debug Panel', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.06 0.08 0.88 0.84]);

annotation(f_dbg, 'textbox', [0.04 0.955 0.92 0.03], ...
    'String', sprintf('Debug panel from %s', file), ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'left', ...
    'Interpreter', 'none', ...
    'FontWeight', 'bold', ...
    'Color', [0.15 0.15 0.15]);

% Top-left: end-effector force in EE/contact frame
pdbg11 = uipanel('Parent', f_dbg, 'Position', getPos(1,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tldbg11 = tiledlayout(pdbg11, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_dbg_preload = nexttile(tldbg11, 1);
plot(time, cmd_force_named, '--', 'LineWidth', 1.00, 'Color', [0.30 0.30 0.30]); hold on;
plot(time, c_hat_fx_act, '-', 'LineWidth', 1.15, 'Color', [0.10 0.35 0.85]);
plot(time, preload_feedback, '-', 'LineWidth', 1.05, 'Color', [0.90 0.25 0.55]);
grid on;
ylabel('normal/preload');
title('Preload / Normal Force');
apply_user_ylim(ax_dbg_preload, nan(2, 2), 1, [cmd_force_named; c_hat_fx_act; preload_feedback]);
legend(ax_dbg_preload, {'cmd force', 'contact force x', 'preload feedback'}, 'Location', 'best');

ax_dbg_t1 = nexttile(tldbg11, 2);
plot(time, c_hat_vy_cmd, '--', 'LineWidth', 1.00, 'Color', [0.85 0.33 0.10]); hold on;
plot(time, c_hat_vy_act, '-', 'LineWidth', 1.15, 'Color', [0.10 0.35 0.85]);
grid on;
ylabel('t1 [m/s]');
title('t1 Command / Actual');
apply_user_ylim(ax_dbg_t1, nan(2, 2), 1, [c_hat_vy_cmd; c_hat_vy_act]);
legend(ax_dbg_t1, {'t1 cmd', 't1 act'}, 'Location', 'best');
apply_panel_xlim([ax_dbg_preload ax_dbg_t1], xlim_debug, 'debug_t1_preload');
xlabel(tldbg11, 'time [s]');

% Top-right: force measured x/y/z in world frame
pdbg12 = uipanel('Parent', f_dbg, 'Position', getPos(1,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tldbg12 = tiledlayout(pdbg12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_dbg_force = gobjects(0);
force_labels = {'F_x', 'F_y', 'F_z'};
for i = 1:3
    ax = nexttile(tldbg12, i); ax_dbg_force(end+1) = ax; %#ok<SAGROW>
    plot(time, contact_raw(:, i), '-', 'LineWidth', 0.95, 'Color', [0.75 0.75 0.75]); hold on;
    plot(time, contact_filt(:, i), '-', 'LineWidth', 1.10, 'Color', [0.10 0.35 0.85]);
    grid on;
    ylabel(sprintf('%s [N]', force_labels{i}));
    title_if_first(i, 'Force Measured (World Frame)');
    if i < 3
        ax.XTickLabel = [];
    end
    apply_user_ylim(ax, nan(3, 2), i, [contact_raw(:, i); contact_filt(:, i)]);
end
apply_panel_xlim(ax_dbg_force, xlim_debug, 'debug_force_world');
xlabel(tldbg12, 'time [s]');
legend(ax_dbg_force(1), {'raw', 'filtered'}, 'Location', 'best');

% Bottom-left: force measured y vs t1 measured
pdbg21 = uipanel('Parent', f_dbg, 'Position', getPos(2,1), 'BackgroundColor', 'w', 'BorderType', 'none');
tldbg21 = tiledlayout(pdbg21, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_dbg_fy_vs_t1 = nexttile(tldbg21, 1);
plot(time, contact_filt(:,2), '-', 'LineWidth', 1.15, 'Color', [0.10 0.35 0.85]); hold on;
yyaxis right;
plot(time, c_hat_vy_act, '-', 'LineWidth', 1.00, 'Color', [0.85 0.33 0.10]);
ax_dbg_fy_vs_t1.YColor = [0.10 0.10 0.10];
yyaxis left;
grid on;
ylabel('F_y [N]');
title('Force Measured y vs t1 Measured');
apply_user_ylim(ax_dbg_fy_vs_t1, nan(2, 2), 1, contact_filt(:,2));
yyaxis right;
ylabel('t1 act [m/s]');
yyaxis left;
legend(ax_dbg_fy_vs_t1, {'force y', 't1 act'}, 'Location', 'best');
apply_panel_xlim(ax_dbg_fy_vs_t1, xlim_debug, 'debug_fy_t1cmd');
xlabel(tldbg21, 'time [s]');
xlabel(tldbg21, 'time [s]');

% Bottom-right: force measured y / t1 command
pdbg22 = uipanel('Parent', f_dbg, 'Position', getPos(2,2), 'BackgroundColor', 'w', 'BorderType', 'none');
tldbg22 = tiledlayout(pdbg22, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_dbg_ratio = nexttile(tldbg22, 1);
plot(time, force_y_over_t1_cmd, '-', 'LineWidth', 1.15, 'Color', [0.90 0.25 0.55]); hold on;
grid on;
ylabel('F_y / t1 cmd');
title('Force Measured y / t1 Command');
apply_user_ylim(ax_dbg_ratio, nan(2, 2), 1, force_y_over_t1_cmd);
legend(ax_dbg_ratio, {'force y / t1 cmd'}, 'Location', 'best');
apply_panel_xlim(ax_dbg_ratio, xlim_debug, 'debug_forcey_over_t1');
xlabel(tldbg22, 'time [s]');

set(findall(f_dbg, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

disp("- Debug panel updated: t1/preload, world-force xyz, force y vs t1 cmd, and force y over t1 cmd");
