%% log_plot.m
% Reads data_logger CSV and builds a comparison dashboard.
% Legacy CSV (46 cols), MOB force CSV (49 cols), MOB vs second-order compare
% CSV (58 cols), MOB vs second-order vs consistency compare CSV (64 cols),
% and consistency-debug CSV (76 cols) are supported

clear; clc;
set(groot, 'defaultFigureRenderer', 'painters');
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');

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

panel_labels = { ...
    'Force XYZ (pyqt style)', ...
    'Overall Panel', ...
    'Debug Panel', ...
    'Normal Estimation', ...
    'Debug Extra'};
[selected_panels, ok] = listdlg( ...
    'PromptString', 'Select panels to open', ...
    'SelectionMode', 'multiple', ...
    'ListString', panel_labels, ...
    'InitialValue', 1:numel(panel_labels), ...
    'ListSize', [240, 150], ...
    'Name', 'log_plot panels');
if ~ok
    disp("Canceled.");
    return;
end
show_force_xyz_panel = any(selected_panels == 1);
show_overall_panel = any(selected_panels == 2);
show_debug_panel = any(selected_panels == 3);
show_normal_estimation_panel = any(selected_panels == 4);
show_debug_extra_panel = any(selected_panels == 5);

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

control_input_world = thrust_world_from_pose_and_fz( ...
    att_pose(:,1), att_pose(:,2), att_pose(:,3), Fz);

pure_force_fallback = nan(size(time, 1), 3);
pure_torque_fallback = nan(size(time, 1), 3);
k_ep_force_fallback = nan(size(time, 1), 3);
k_ep_torque_fallback = nan(size(time, 1), 3);
k_epi_force_fallback = nan(size(time, 1), 3);
kalman_force_fallback = nan(size(time, 1), 3);
adaptive_force_fallback = nan(size(time, 1), 3);
if mob_compare_valid
    pure_force_fallback = -mob1_force;
    pure_torque_fallback = -mob1_torque;
    k_ep_force_fallback = -mob2_force;
    k_ep_torque_fallback = -mob2_torque;
elseif mob_force_valid
    pure_force_fallback = -mob_force;
end

force_panel_pure = [
    -get_first_matching_column(T, {'mob_2nd_order_Fx', 'pure_Fx'}, -pure_force_fallback(:,1)), ...
    -get_first_matching_column(T, {'mob_2nd_order_Fy', 'pure_Fy'}, -pure_force_fallback(:,2)), ...
    -get_first_matching_column(T, {'mob_2nd_order_Fz', 'pure_Fz'}, -pure_force_fallback(:,3))];
force_panel_k_ep = [
    -get_first_matching_column(T, {'mob_2nd_tau_Fx', 'k_ep_Fx'}, -k_ep_force_fallback(:,1)), ...
    -get_first_matching_column(T, {'mob_2nd_tau_Fy', 'k_ep_Fy'}, -k_ep_force_fallback(:,2)), ...
    -get_first_matching_column(T, {'mob_2nd_tau_Fz', 'k_ep_Fz'}, -k_ep_force_fallback(:,3))];
force_panel_pure_torque = [
    -get_first_matching_column(T, {'mob_2nd_order_Tx', 'pure_Tx'}, -pure_torque_fallback(:,1)), ...
    -get_first_matching_column(T, {'mob_2nd_order_Ty', 'pure_Ty'}, -pure_torque_fallback(:,2)), ...
    -get_first_matching_column(T, {'mob_2nd_order_Tz', 'pure_Tz'}, -pure_torque_fallback(:,3))];
force_panel_k_ep_torque = [
    -get_first_matching_column(T, {'mob_2nd_tau_Tx', 'k_ep_Tx'}, -k_ep_torque_fallback(:,1)), ...
    -get_first_matching_column(T, {'mob_2nd_tau_Ty', 'k_ep_Ty'}, -k_ep_torque_fallback(:,2)), ...
    -get_first_matching_column(T, {'mob_2nd_tau_Tz', 'k_ep_Tz'}, -k_ep_torque_fallback(:,3))];
force_panel_k_epi = [
    -get_first_matching_column(T, {'mob_2nd_tau_i_Fx', 'k_epi_Fx'}, -k_epi_force_fallback(:,1)), ...
    -get_first_matching_column(T, {'mob_2nd_tau_i_Fy', 'k_epi_Fy'}, -k_epi_force_fallback(:,2)), ...
    -get_first_matching_column(T, {'mob_2nd_tau_i_Fz', 'k_epi_Fz'}, -k_epi_force_fallback(:,3))];
force_panel_kalman = [
    -get_first_matching_column(T, {'mob_kalman_Fx', 'kalman_Fx'}, -kalman_force_fallback(:,1)), ...
    -get_first_matching_column(T, {'mob_kalman_Fy', 'kalman_Fy'}, -kalman_force_fallback(:,2)), ...
    -get_first_matching_column(T, {'mob_kalman_Fz', 'kalman_Fz'}, -kalman_force_fallback(:,3))];
force_panel_adaptive = [
    -get_first_matching_column(T, {'mob_adaptive_Fx', 'adaptive_Fx'}, -adaptive_force_fallback(:,1)), ...
    -get_first_matching_column(T, {'mob_adaptive_Fy', 'adaptive_Fy'}, -adaptive_force_fallback(:,2)), ...
    -get_first_matching_column(T, {'mob_adaptive_Fz', 'adaptive_Fz'}, -adaptive_force_fallback(:,3))];
force_panel_has_k_epi = any(isfinite(force_panel_k_epi(:)));
force_panel_has_kalman = any(isfinite(force_panel_kalman(:)));
force_panel_has_adaptive = any(isfinite(force_panel_adaptive(:)));
force_panel_true_wind = [
    -get_column_by_name(T, "wind_x", nan(size(time))), ...
    -get_column_by_name(T, "wind_y", nan(size(time))), ...
    -get_column_by_name(T, "wind_z", nan(size(time)))];
k_ep_sweep_gains = [100 300 500];
force_panel_k_ep_sweep = nan(size(time, 1), 3, numel(k_ep_sweep_gains));
force_panel_k_ep_torque_sweep = nan(size(time, 1), 3, numel(k_ep_sweep_gains));
for gain_idx = 1:numel(k_ep_sweep_gains)
    gain = k_ep_sweep_gains(gain_idx);
    force_panel_k_ep_sweep(:, :, gain_idx) = [
        -get_first_matching_column(T, gain_column_candidates(gain, 'Fx'), nan(size(time))), ...
        -get_first_matching_column(T, gain_column_candidates(gain, 'Fy'), nan(size(time))), ...
        -get_first_matching_column(T, gain_column_candidates(gain, 'Fz'), nan(size(time)))];
    force_panel_k_ep_torque_sweep(:, :, gain_idx) = [
        -get_first_matching_column(T, gain_column_candidates(gain, 'Tx'), nan(size(time))), ...
        -get_first_matching_column(T, gain_column_candidates(gain, 'Ty'), nan(size(time))), ...
        -get_first_matching_column(T, gain_column_candidates(gain, 'Tz'), nan(size(time)))];
end
force_panel_base_gain_idx = find(k_ep_sweep_gains == 500, 1);
if ~isempty(force_panel_base_gain_idx) && ...
        ~any(isfinite(reshape(force_panel_k_ep_sweep(:, :, force_panel_base_gain_idx), [], 1)))
    force_panel_k_ep_sweep(:, :, force_panel_base_gain_idx) = force_panel_k_ep;
end
if ~isempty(force_panel_base_gain_idx) && ...
        ~any(isfinite(reshape(force_panel_k_ep_torque_sweep(:, :, force_panel_base_gain_idx), [], 1)))
    force_panel_k_ep_torque_sweep(:, :, force_panel_base_gain_idx) = force_panel_k_ep_torque;
end
force_panel_ee_offset_body = [0.09, 0.0, 0.085];
force_panel_r_world = rotate_body_vectors(att_pose(:,1), att_pose(:,2), att_pose(:,3), force_panel_ee_offset_body);
force_panel_residual_pure = subtract_rows(force_panel_pure_torque, cross_rows(force_panel_r_world, force_panel_pure));
force_panel_residual_k_ep_sweep = nan(size(force_panel_k_ep_sweep));
for gain_idx = 1:numel(k_ep_sweep_gains)
    force_panel_residual_k_ep_sweep(:, :, gain_idx) = subtract_rows( ...
        force_panel_k_ep_torque_sweep(:, :, gain_idx), ...
        cross_rows(force_panel_r_world, force_panel_k_ep_sweep(:, :, gain_idx)));
end
force_panel_angle_error_pure = vector_direction_error_deg(force_panel_true_wind, force_panel_pure);
force_panel_angle_error_k_ep_sweep = nan(size(time, 1), numel(k_ep_sweep_gains));
for gain_idx = 1:numel(k_ep_sweep_gains)
    force_panel_angle_error_k_ep_sweep(:, gain_idx) = ...
        vector_direction_error_deg(force_panel_true_wind, force_panel_k_ep_sweep(:, :, gain_idx));
end
force_panel_angle_signals = [force_panel_angle_error_pure, force_panel_angle_error_k_ep_sweep];
force_panel_angle_nan_start_idx = find_nan_onset_after_valid(force_panel_angle_signals);
if isempty(force_panel_angle_nan_start_idx)
    force_panel_angle_auto_xlim = [time(1) time(end)];
else
    force_panel_angle_nan_start_time = time(force_panel_angle_nan_start_idx);
    force_panel_angle_auto_xlim = [
        max(time(1), force_panel_angle_nan_start_time - 3.0), ...
        min(time(end), force_panel_angle_nan_start_time + 4.0)];
end


%% Normal estimation panel
if show_normal_estimation_panel
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
end


%% Debug extra figure
if show_debug_extra_panel
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
end

%% Force XYZ figure
if show_force_xyz_panel
force_panel_xlim = [15.91 25.91];
force_panel_angle_xlim = force_panel_xlim;
% force_panel_angle_xlim = force_panel_angle_auto_xlim;
force_panel_force_ylims = [
    -0.02 0.05;
    -0.02 0.05;
    -0.02 0.05];
force_panel_residual_ylims = [
    -0.001 0.005;
    -0.001 0.005;
    -0.001 0.005];
force_panel_angle_error_ylims = [nan nan];
force_panel_window_size = [520 860];
force_panel_line_width = 1.4;
force_panel_axis_names = {'x', 'y', 'z'};
force_panel_force_labels = {'ʷf̂ₗ,ₓ [N]', 'ʷf̂ₗ,ᵧ [N]', 'ʷf̂ₗ,z [N]'};
force_panel_torque_labels = {'ʷτ̂ₗ,ₓ [N m]', 'ʷτ̂ₗ,ᵧ [N m]', 'ʷτ̂ₗ,z [N m]'};
force_panel_true_color = [0.05 0.05 0.05];
force_panel_pure_color = [0.16 0.35 0.86];
force_panel_k_ep_colors = [
    0.82 0.18 0.18;
    0.00 0.52 0.62;
    0.88 0.62 0.08];
force_panel_pure_style = '--';
force_panel_pure_width = 1.6;
force_panel_k_ep_styles = {'-.', ':', '--'};
force_panel_k_ep_widths = [1.6 1.9 1.6];
force_panel_consistency_color = [0.20 0.20 0.20];
force_panel_consistency_width = 1.4;
force_panel_time = time - force_panel_xlim(1);
force_panel_display_xlim = [0, force_panel_xlim(2) - force_panel_xlim(1)];

f_force = figure('Name', 'Force XYZ Panel', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'pixels', 'Position', [680 80 900 force_panel_window_size(2)]);
tlf = tiledlayout(f_force, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
tlf.Units = 'normalized';
tlf.Position = [0.075 0.065 0.85 0.685];

ax_force_left = gobjects(0);
ax_force_right = gobjects(0);
force_panel_left_handles = gobjects(0);
force_panel_right_handles = gobjects(0);
for i = 1:3
    ax = nexttile(tlf, 2 * i - 1); ax_force_left(end+1) = ax; %#ok<SAGROW>
    hold(ax, 'on');
    h_actual = plot(ax, force_panel_time, force_panel_true_wind(:, i), '-', 'LineWidth', 2.1, 'Color', force_panel_true_color);
    h_pure = plot(ax, force_panel_time, force_panel_pure(:, i), force_panel_pure_style, ...
        'LineWidth', force_panel_pure_width, 'Color', force_panel_pure_color);
    h_kep = gobjects(1, numel(k_ep_sweep_gains));
    for gain_idx = 1:numel(k_ep_sweep_gains)
        h_kep(gain_idx) = plot(ax, force_panel_time, force_panel_k_ep_sweep(:, i, gain_idx), force_panel_k_ep_styles{gain_idx}, ...
            'LineWidth', force_panel_k_ep_widths(gain_idx), ...
            'Color', force_panel_k_ep_colors(gain_idx, :));
    end
    if i == 1
        force_panel_left_handles = [h_actual, h_pure, h_kep];
    end
    grid(ax, 'on');
    ylabel(ax, force_panel_force_labels{i}, 'Interpreter', 'none');
    title_if_first(i, 'Actual vs Estimated EE Force');
    if i < 3
        ax.XTickLabel = [];
    end
    force_panel_stack = [force_panel_true_wind(:, i); force_panel_pure(:, i)];
    for gain_idx = 1:numel(k_ep_sweep_gains)
        force_panel_stack = [force_panel_stack; force_panel_k_ep_sweep(:, i, gain_idx)]; %#ok<AGROW>
    end
    apply_user_ylim(ax, force_panel_force_ylims, i, force_panel_stack);

    ax = nexttile(tlf, 2 * i); ax_force_right(end+1) = ax; %#ok<SAGROW>
    hold(ax, 'on');
    yline(ax, 0.0, ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    h_residual_kep = gobjects(1, numel(k_ep_sweep_gains));
    for gain_idx = 1:numel(k_ep_sweep_gains)
        h_residual_kep(gain_idx) = plot(ax, force_panel_time, force_panel_residual_k_ep_sweep(:, i, gain_idx), ...
            force_panel_k_ep_styles{gain_idx}, ...
            'LineWidth', force_panel_k_ep_widths(gain_idx), ...
            'Color', force_panel_k_ep_colors(gain_idx, :));
    end
    h_residual_pure = plot(ax, force_panel_time, force_panel_residual_pure(:, i), force_panel_pure_style, ...
        'LineWidth', force_panel_pure_width, 'Color', force_panel_pure_color);
    if i == 1
        force_panel_right_handles = [h_residual_pure, h_residual_kep];
    end
    grid(ax, 'on');
    ylabel(ax, force_panel_torque_labels{i}, 'Interpreter', 'none');
    title_if_first(i, 'Consistency Residual');
    if i < 3
        ax.XTickLabel = [];
    end
    force_panel_residual_stack = force_panel_residual_pure(:, i);
    for gain_idx = 1:numel(k_ep_sweep_gains)
        force_panel_residual_stack = [force_panel_residual_stack; force_panel_residual_k_ep_sweep(:, i, gain_idx)]; %#ok<AGROW>
    end
    apply_user_ylim(ax, force_panel_residual_ylims, i, force_panel_residual_stack);
end
apply_panel_xlim([ax_force_left ax_force_right], force_panel_display_xlim, 'force_xyz_panel');
xlabel(tlf, 'time [s]');
legend_labels = {'actual', 'pure'};
for gain_idx = 1:numel(k_ep_sweep_gains)
    legend_labels{end+1} = sprintf('k\\_ep %d', k_ep_sweep_gains(gain_idx)); %#ok<SAGROW>
end
lgd_force_left = legend(ax_force_left(1), force_panel_left_handles, legend_labels, 'Location', 'northeastoutside');
lgd_force_right = legend(ax_force_right(1), force_panel_right_handles, legend_labels(2:end), 'Location', 'northeastoutside');
drawnow;
lgd_force_left.Units = 'normalized';
lgd_force_right.Units = 'normalized';
lgd_force_left.Position(1) = 0.30;
lgd_force_left.Position(2) = 0.945;
lgd_force_right.Position(1) = 0.79;
lgd_force_right.Position(2) = 0.945;
set(findall(f_force, '-property', 'FontName'), 'FontName', 'Times New Roman');

force_panel_angle_mask = isfinite(time) & ...
    (time >= force_panel_angle_xlim(1)) & (time <= force_panel_angle_xlim(2));
force_panel_angle_time = time(force_panel_angle_mask) - force_panel_angle_xlim(1);
force_panel_angle_error_pure_window = force_panel_angle_error_pure(force_panel_angle_mask);
force_panel_angle_error_k_ep_window = force_panel_angle_error_k_ep_sweep(force_panel_angle_mask, :);

force_panel_angle_figure = figure('Name', 'Force Direction Angle Error', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'pixels', 'Position', [1610 80 520 400]);
ax_force_angle = axes(force_panel_angle_figure);
ax_force_angle.Units = 'normalized';
ax_force_angle.Position = [0.12 0.12 0.78 0.56];
hold(ax_force_angle, 'on');
plot(ax_force_angle, force_panel_angle_time, zeros(size(force_panel_angle_time)), '-', ...
    'LineWidth', 2.1, 'Color', force_panel_true_color);
plot(ax_force_angle, force_panel_angle_time, force_panel_angle_error_pure_window, force_panel_pure_style, ...
    'LineWidth', force_panel_pure_width, 'Color', force_panel_pure_color);
for gain_idx = 1:numel(k_ep_sweep_gains)
    plot(ax_force_angle, force_panel_angle_time, force_panel_angle_error_k_ep_window(:, gain_idx), ...
        force_panel_k_ep_styles{gain_idx}, ...
        'LineWidth', force_panel_k_ep_widths(gain_idx), ...
        'Color', force_panel_k_ep_colors(gain_idx, :));
end
grid(ax_force_angle, 'on');
ylabel(ax_force_angle, 'angle error [deg]');
xlabel(ax_force_angle, 'time [s]');
title(ax_force_angle, 'Force Direction Angle Error vs Actual');
apply_user_ylim(ax_force_angle, force_panel_angle_error_ylims, 1, ...
    [zeros(size(force_panel_angle_time)); force_panel_angle_error_pure_window; reshape(force_panel_angle_error_k_ep_window, [], 1)]);
apply_panel_xlim(ax_force_angle, [0, force_panel_angle_xlim(2) - force_panel_angle_xlim(1)], 'force_direction_angle_error');
lgd_force_angle = legend(ax_force_angle, legend_labels, 'Location', 'northeastoutside');
drawnow;
lgd_force_angle.Units = 'normalized';
lgd_force_angle.Position(1) = 0.61;
lgd_force_angle.Position(2) = 0.93;
set(findall(force_panel_angle_figure, '-property', 'FontName'), 'FontName', 'Times New Roman');
end


%% Overall figure only
if show_overall_panel
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
end

%% Debug figure
if show_debug_panel
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
end

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

function world_vectors = rotate_body_vectors(roll, pitch, yaw, body_vector)
    n = numel(roll);
    world_vectors = nan(n, 3);
    if numel(pitch) ~= n || numel(yaw) ~= n || numel(body_vector) ~= 3
        return;
    end
    body_vector = reshape(body_vector, [3, 1]);
    for k = 1:n
        if ~all(isfinite([roll(k), pitch(k), yaw(k)]))
            continue;
        end
        cr = cos(roll(k)); sr = sin(roll(k));
        cp = cos(pitch(k)); sp = sin(pitch(k));
        cy = cos(yaw(k)); sy = sin(yaw(k));
        r_bw = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
                sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
                -sp,   cp*sr,            cp*cr];
        world_vectors(k, :) = (r_bw * body_vector).';
    end
end

function c = cross_rows(a, b)
    c = nan(size(a));
    if ~isequal(size(a), size(b)) || size(a, 2) ~= 3
        return;
    end
    valid = all(isfinite(a), 2) & all(isfinite(b), 2);
    if ~any(valid)
        return;
    end
    c(valid, :) = cross(a(valid, :), b(valid, :), 2);
end

function out = subtract_rows(a, b)
    out = nan(size(a));
    if ~isequal(size(a), size(b))
        return;
    end
    valid = all(isfinite(a), 2) & all(isfinite(b), 2);
    out(valid, :) = a(valid, :) - b(valid, :);
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

function out = get_first_matching_column(T, names, fallback)
    out = fallback;
    for i = 1:numel(names)
        name = names{i};
        if ismember(name, T.Properties.VariableNames)
            candidate = T.(name);
            if iscolumn(candidate) && numel(candidate) == numel(fallback)
                out = candidate;
                return;
            end
        end
    end
end

function names = gain_column_candidates(display_gain, axis_suffix)
    switch display_gain
        case 50
            source_gains = [50, 100];
        case 150
            source_gains = [150, 100, 300];
        case 300
            source_gains = [300, 500];
        otherwise
            source_gains = display_gain;
    end
    names = cell(1, numel(source_gains));
    for i = 1:numel(source_gains)
        names{i} = sprintf('mob_2nd_tau_ke_%d_%s', source_gains(i), axis_suffix);
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

function angle_deg = vector_direction_error_deg(ref_vectors, target_vectors)
    angle_deg = nan(size(ref_vectors, 1), 1);
    if ~isequal(size(ref_vectors), size(target_vectors)) || size(ref_vectors, 2) ~= 3
        return;
    end
    ref_norm = vecnorm(ref_vectors, 2, 2);
    target_norm = vecnorm(target_vectors, 2, 2);
    valid = all(isfinite(ref_vectors), 2) & all(isfinite(target_vectors), 2) & ...
        (ref_norm > 1.0e-9) & (target_norm > 1.0e-9);
    if ~any(valid)
        return;
    end
    cos_theta = sum(ref_vectors(valid, :) .* target_vectors(valid, :), 2) ./ ...
        (ref_norm(valid) .* target_norm(valid));
    cos_theta = max(-1.0, min(1.0, cos_theta));
    angle_deg(valid) = acosd(cos_theta);
end

function idx = find_nan_onset_after_valid(signals)
    idx = [];
    if isempty(signals)
        return;
    end
    has_valid = any(isfinite(signals), 2);
    all_nan = all(~isfinite(signals), 2);
    transition_idx = find(has_valid(1:end-1) & all_nan(2:end), 1, 'first');
    if ~isempty(transition_idx)
        idx = transition_idx + 1;
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
