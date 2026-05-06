%% log_plot.m
% Reads data_logger CSV and builds a comparison dashboard.
% Legacy CSV (46 cols), MOB force CSV (49 cols), MOB vs second-order compare
% CSV (58 cols), MOB vs second-order vs consistency compare CSV (64 cols),
% consistency-debug CSV (76 cols), and wind-augmented CSV (79 cols) are supported.

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
vars = string(T.Properties.VariableNames);

if size(A, 2) < 46
    error("CSV must have at least 46 columns, got %d", size(A, 2));
end

col = @(k) A(:, k);
get_any = @(names) local_get_any(T, vars, names);
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
    mob2_force = [col(47), col(48), col(49)];
    mob2_torque = [col(50), col(51), col(52)];
    mob3_force = [col(53), col(54), col(55)];
    mob3_torque = [col(56), col(57), col(58)];
    mob_compare_valid = true;
else
    z3 = zeros(size(A,1), 3);
    mob2_force = z3; mob2_torque = z3;
    mob3_force = z3; mob3_torque = z3;
    mob_compare_valid = false;
    warning('MOB and second-order force/torque columns not found. Re-log with updated data_logger for 3x2 MOB compare figure.');
end

if size(A, 2) >= 70
    mob_tau_kfep = [col(59), col(60), col(61)];
    mob_tau_consistency = [col(62), col(63), col(64)];
    mob_tau_tauhat = [col(65), col(66), col(67)];
    mob_tau_rxf = [col(68), col(69), col(70)];
    mob_tau_debug_valid = true;
else
    z3 = zeros(size(A,1), 3);
    mob_tau_kfep = z3;
    mob_tau_consistency = z3;
    mob_tau_tauhat = z3;
    mob_tau_rxf = z3;
    mob_tau_debug_valid = false;
end

if size(A, 2) >= 73
    wind_force = [col(71), col(72), col(73)];
    wind_valid = true;
else
    wind_force = zeros(size(A,1), 3);
    wind_valid = false;
end

if size(A, 2) >= 93
    ee_pos = [col(74), col(75), col(76)];
    c_hat_vy_cmd = col(77);
    c_hat_vy_act = col(78);
    c_hat_vz_cmd = col(79);
    c_hat_vz_act = col(80);
    c_hat_fx_act = col(81);
    n_geo = [col(82), col(83), col(84)];
    n_f = [col(85), col(86), col(87)];
    n_alg = [col(88), col(89), col(90)];
    f_g = [col(91), col(92), col(93)];
    normal_force_panel_valid = true;
else
    ee_pos = nan(size(A, 1), 3);
    c_hat_vy_cmd = nan(size(A, 1), 1);
    c_hat_vy_act = nan(size(A, 1), 1);
    c_hat_vz_cmd = nan(size(A, 1), 1);
    c_hat_vz_act = nan(size(A, 1), 1);
    c_hat_fx_act = nan(size(A, 1), 1);
    n_geo = nan(size(A, 1), 3);
    n_f = nan(size(A, 1), 3);
    n_alg = nan(size(A, 1), 3);
    f_g = nan(size(A, 1), 3);
    normal_force_panel_valid = false;
end

%% ---- optional offline-style comparison fields ----
mobc_force = [ ...
    get_any(["offline_mobc_fx"]), ...
    get_any(["offline_mobc_fy"]), ...
    get_any(["offline_mobc_fz"])];
mobc_torque = [ ...
    get_any(["offline_mobc_tx"]), ...
    get_any(["offline_mobc_ty"]), ...
    get_any(["offline_mobc_tz"])];
normal_est_pure = [ ...
    get_any(["offline_normal_pure_nx"]), ...
    get_any(["offline_normal_pure_ny"]), ...
    get_any(["offline_normal_pure_nz"])];
normal_est_ke = [ ...
    get_any(["offline_normal_k1_nx", "offline_normal_ke_nx"]), ...
    get_any(["offline_normal_k1_ny", "offline_normal_ke_ny"]), ...
    get_any(["offline_normal_k1_nz", "offline_normal_ke_nz"])];
normal_est_ke_raw = [ ...
    get_any(["offline_normal_ke_raw_nx"]), ...
    get_any(["offline_normal_ke_raw_ny"]), ...
    get_any(["offline_normal_ke_raw_nz"])];
normal_est_ke_gamma_proj = [ ...
    get_any(["offline_normal_ke_gamma_proj_nx"]), ...
    get_any(["offline_normal_ke_gamma_proj_ny"]), ...
    get_any(["offline_normal_ke_gamma_proj_nz"])];
w_s_logged = [ ...
    get_any(["offline_ws_x"]), ...
    get_any(["offline_ws_y"]), ...
    get_any(["offline_ws_z"])];
gamma_v_logged = get_any(["offline_gamma_v"]);
normal_est_vel_eps0 = [ ...
    get_any(["offline_normal_vel_eps0_nx"]), ...
    get_any(["offline_normal_vel_eps0_ny"]), ...
    get_any(["offline_normal_vel_eps0_nz"])];
normal_est_vel_eps = [ ...
    get_any(["offline_normal_vel_eps_nx"]), ...
    get_any(["offline_normal_vel_eps_ny"]), ...
    get_any(["offline_normal_vel_eps_nz"])];
normal_est_vel_eps0_gamma = [ ...
    get_any(["offline_normal_vel_eps0_gamma_nx"]), ...
    get_any(["offline_normal_vel_eps0_gamma_ny"]), ...
    get_any(["offline_normal_vel_eps0_gamma_nz"])];

if ~any(isfinite(mobc_force(:)))
    mobc_force = mob3_force;
    mobc_torque = mob3_torque;
end
%% ---- comparison signals ----
mass_nominal = 0.04338;
mass_acc = mass_nominal .* acc_xyz;
force_world_default_cmp = thrust_world_from_pose_and_fz(att_pose(:,1), att_pose(:,2), att_pose(:,3), Fz);
force_world_default_cmp(:,3) = force_world_default_cmp(:,3) - mass_nominal * 9.81;
ee_offset_body = [0.1, 0.0, 0.04];
environment_type = "wall";
wall_x = 0.2;
cylinder_center = [1.5, 0.0, 0.0];

[~, r_world] = thrust_world_from_pose_and_fz(att_pose(:,1), att_pose(:,2), att_pose(:,3), Fz, ee_offset_body);
rxf_pure = cross_rows(r_world, mob2_force);
rxf_consistency = cross_rows(r_world, mobc_force);
n_gt = estimate_gt_normal(ee_pos, environment_type, wall_x, cylinder_center);
angle_n_geo_deg = angle_deg_rows(n_gt, normalize_rows(n_geo));
angle_n_f_deg = angle_deg_rows(n_gt, normalize_rows(n_f));
angle_n_alg_deg = angle_deg_rows(n_gt, normalize_rows(n_alg));

normal_est_pure = normalize_rows(normal_est_pure);
normal_est_ke = normalize_rows(normal_est_ke);
normal_est_ke_raw = normalize_rows(normal_est_ke_raw);
normal_est_ke_gamma_proj = normalize_rows(normal_est_ke_gamma_proj);
normal_est_vel_eps0 = normalize_rows(normal_est_vel_eps0);
normal_est_vel_eps = normalize_rows(normal_est_vel_eps);
normal_est_vel_eps0_gamma = normalize_rows(normal_est_vel_eps0_gamma);
if ~any(isfinite(normal_est_ke(:))) && normal_force_panel_valid
    normal_est_ke = normalize_rows(n_geo);
end
if ~any(isfinite(normal_est_ke_raw(:))) && normal_force_panel_valid
    normal_est_ke_raw = normalize_rows(n_f);
end
if ~any(isfinite(normal_est_ke_raw(:)))
    normal_est_ke_raw = normalize_rows(-mobc_force);
end
ke_force_ee_applied = -mobc_force;
if ~any(isfinite(w_s_logged(:)))
    w_s_logged = normalize_rows(vel_xyz);
end
if ~any(isfinite(gamma_v_logged))
    speed_for_gamma = sqrt(sum(vel_xyz.^2, 2));
    gamma_v_logged = (speed_for_gamma.^2) ./ (speed_for_gamma.^2 + 1.0e-2);
end
if ~any(isfinite(normal_est_ke_gamma_proj(:)))
    normal_est_ke_gamma_proj = normalize_rows(project_rows_with_ws_gamma(ke_force_ee_applied, normalize_rows(vel_xyz), gamma_v_logged));
end
if ~any(isfinite(normal_est_vel_eps0(:)))
    normal_est_vel_eps0 = normalize_rows(project_rows_with_ws(ke_force_ee_applied, normalize_rows(vel_xyz), true));
end
if ~any(isfinite(normal_est_vel_eps(:)))
    normal_est_vel_eps = normalize_rows(project_rows_with_ws(ke_force_ee_applied, w_s_logged));
end
if ~any(isfinite(normal_est_vel_eps0_gamma(:)))
    normal_est_vel_eps0_gamma = normalize_rows(project_rows_with_ws_gamma(ke_force_ee_applied, normalize_rows(vel_xyz), gamma_v_logged));
end
angle_true_pure_deg = angle_deg_rows(n_gt, normal_est_pure);
angle_true_ke_deg = angle_deg_rows(n_gt, normal_est_ke);
mob2_force_norm = sqrt(sum(mob2_force.^2, 2));
mobc_force_norm = sqrt(sum(mobc_force.^2, 2));
cmd_xyz = cmd_xyzyaw(:, 1:3);
att_cmd = att_des_u;

pos_names = {'X', 'Y', 'Z'};
axis_names = {'X', 'Y', 'Z'};
att_names = {'roll', 'pitch', 'yaw'};
pure_color = [0.0000 0.4470 0.7410];
cons_color = [0.8500 0.3250 0.0980];
tau_color = [0.4660 0.6740 0.1880];
meas_color = [0.6350 0.0780 0.1840];
cmd_color = [0.0000 0.4470 0.7410];
rbf_color = [0.4940 0.1840 0.5560];
lw1 = 1.2;
lw2 = 1.15;
lw_main = lw2;
lw_wind = 2.2;

disp("Done. Variables prepared:");
disp("- pose_xyz / cmd_xyzyaw");
disp("- mass_acc / force_world_default_cmp");
disp("- att_pose / att_des_u");
disp("- mob2_force / mob3_force");
disp("- mob_tau_tauhat / rxf_pure / rxf_consistency");

%% Panel 0) Position / s / gamma_v and projected-normal comparisons
xlim_panel0 = [30, 70];
ylim_panel0_pos = [nan nan; nan nan; nan nan];
ylim_panel0_vel = [nan nan; nan nan; nan nan];
ylim_panel0_ws = [-1.1 1.1; -1.1 1.1; -1.1 1.1];
ylim_panel0_normal = [-0.1 2.1; -1.1 1.1; -1.1 1.1];
true_wall_normal = repmat([1.0, 0.0, 0.0], size(time, 1), 1);

f0 = figure('Name','Panel 0 - Position, s, gamma_v, and projected-normal comparisons','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.05 0.05 0.90 0.88]);

p_panel0_vel = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.03 0.54 0.29 0.41], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
plot_cmd_meas_triplet_panel(p_panel0_vel, time, cmd_xyz, pose_xyz, pos_names, ...
    'command', cmd_color, 'measured', meas_color, 'Position command vs measured', '[m]', ...
    ylim_panel0_pos, xlim_panel0, 1.25, lw2);

p_panel0_ws = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.35 0.54 0.29 0.41], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
plot_single_triplet_panel(p_panel0_ws, time, w_s_logged, pos_names, ...
    [0.10 0.55 0.20], 's = v / |v|', '[-]', ylim_panel0_ws, xlim_panel0, lw2);

p_panel0_gamma = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.67 0.54 0.29 0.41], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
plot_scalar_or_message(p_panel0_gamma, time, gamma_v_logged, [0.10 0.45 0.95], ...
    '\gamma_v', '[-]', [0 1.05], xlim_panel0, any(isfinite(gamma_v_logged)), ...
    '\gamma_v is not logged in this CSV');

p_panel0_ke = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.03 0.07 0.29 0.41], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_or_message(p_panel0_ke, time, true_wall_normal, normal_est_ke_raw, axis_names, ...
    'true normal (1,0,0)', [0.15 0.15 0.15], 'without projection', cons_color, ...
    'True normal vs without projection', '[-]', lw1, lw2, ylim_panel0_normal, xlim_panel0, ...
    any(isfinite(normal_est_ke_raw(:))), 'without-projection normal is not logged in this CSV');

p_panel0_ke_proj = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.35 0.07 0.29 0.41], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_or_message(p_panel0_ke_proj, time, true_wall_normal, normal_est_vel_eps, axis_names, ...
    'true normal (1,0,0)', [0.15 0.15 0.15], 'with projection (\epsilon_v)', tau_color, ...
    'True normal vs with projection (velocity\_epsilon)', '[-]', lw1, lw2, ylim_panel0_normal, xlim_panel0, ...
    any(isfinite(normal_est_vel_eps(:))), 'velocity\_epsilon-aware normal is not logged in this CSV');

p_panel0_proj_raw = uipanel('Parent', f0, 'Units', 'normalized', 'Position', [0.67 0.07 0.29 0.41], ...
    'BackgroundColor', 'w', 'BorderType', 'none');
plot_compare_triplet_or_message(p_panel0_proj_raw, time, true_wall_normal, normal_est_ke_gamma_proj, axis_names, ...
    'true normal (1,0,0)', [0.15 0.15 0.15], 'with projection (\gamma_v)', [0.10 0.45 0.95], ...
    'True normal vs with projection (\gamma_v)', '[-]', lw1, lw2, ylim_panel0_normal, xlim_panel0, ...
    any(isfinite(normal_est_ke_gamma_proj(:))), '\gamma_v-projected normal is not logged in this CSV');

set(findall(f0, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Panel 1) Y-axis focused MOB compare
xlim_panel1_row1 = [5.5 25.5];
xlim_panel1_row2 = [5.5 25.5];
ylim_panel1_row1 = repmat([-0.005 0.025], 3, 1);
ylim_panel1_row2 = repmat([-0.005 0.005], 3, 1);

f1 = figure('Name','Panel 1 - Y-axis MOB compare','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.05 0.05 0.90 0.88]);
tl1 = tiledlayout(f1, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_panel1_row1 = gobjects(0);
ax_panel1_row2 = gobjects(0);

panel1_colors.mob2 = [0.20 0.50 0.95];
panel1_colors.mobc = [0.90 0.30 0.18];
panel1_colors.wind = [0.15 0.60 0.30];
panel1_colors.tauhat = [0.45 0.20 0.75];
panel1_colors.rxf_pure = [0.20 0.50 0.95];
panel1_colors.rxf_consistency = [0.90 0.30 0.18];

row1_series = {
    {wind_force(:,2), mob2_force(:,2)}
    {wind_force(:,2), mob2_force(:,2)}
    {wind_force(:,2), mob2_force(:,2), mob3_force(:,2)}
};
row1_labels = {
    {'Wind external', 'MOB 2nd order'}
    {'Wind external', 'MOB 2nd order'}
    {'Wind external', 'MOB 2nd order', 'Consistency residual'}
};
row1_colors = {
    {panel1_colors.wind, panel1_colors.mob2}
    {panel1_colors.wind, panel1_colors.mob2}
    {panel1_colors.wind, panel1_colors.mob2, panel1_colors.mobc}
};
row1_titles = {'MOB 2nd vs wind (Y)', 'MOB 2nd vs wind (Y)', 'MOB 2nd / consistency / wind (Y)'};

for col_idx = 1:3
    ax = nexttile(tl1, col_idx); ax_panel1_row1(end+1) = ax; %#ok<SAGROW>
    hold on;
    series_list = row1_series{col_idx};
    color_list = row1_colors{col_idx};
    label_list = row1_labels{col_idx};
    stacked = [];
    for k = 1:numel(series_list)
        line_width = lw_main;
        if isequal(color_list{k}, panel1_colors.wind)
            line_width = lw_wind;
        end
        plot(time, series_list{k}, '-', 'LineWidth', line_width, 'Color', color_list{k});
        stacked = [stacked; series_list{k}]; %#ok<AGROW>
    end
    grid on;
    apply_user_ylim(ax, ylim_panel1_row1, col_idx, stacked);
    ylabel('force [N]');
    title(row1_titles{col_idx});
    legend(label_list, 'Location', 'best');
    xlabel('time [s]');
end

for i = 1:3
    ax = nexttile(tl1, 3 + i); ax_panel1_row2(end+1) = ax; %#ok<SAGROW>
    plot(time, mob_tau_tauhat(:,i), '-', 'LineWidth', lw_main, 'Color', panel1_colors.tauhat); hold on;
    plot(time, rxf_pure(:,i), '-', 'LineWidth', lw_main, 'Color', panel1_colors.rxf_pure);
    plot(time, rxf_consistency(:,i), '-', 'LineWidth', lw_main, 'Color', panel1_colors.rxf_consistency);
    grid on;
    apply_user_ylim(ax, ylim_panel1_row2, i, [mob_tau_tauhat(:,i); rxf_pure(:,i); rxf_consistency(:,i)]);
    ylabel('torque [Nm]');
    title(sprintf('\\tau ext / r \\times F %s', axis_names{i}));
    if i == 1
        legend({'\tau ext', 'r \times F (pure MOB F)', 'r \times F (consistency MOB F)'}, ...
            'Location', 'best', 'Interpreter', 'tex');
    end
    xlabel('time [s]');
end

apply_panel_xlim(ax_panel1_row1, xlim_panel1_row1, 'panel1_row1_xlink');
apply_panel_xlim(ax_panel1_row2, xlim_panel1_row2, 'panel1_row2_xlink');
set(findall(f1, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Panel 2) cy / cz / Fx + normal-angle / wind
xlim_panel2_row1 = [15 60];
xlim_panel2_row2 = [15 60];
ylim_panel2_row1 = [nan nan; nan nan; nan nan];
ylim_panel2_angle = [0 45];
ylim_panel2_wind = [nan nan];

cy_des = c_hat_vy_cmd;
cy_meas = c_hat_vy_act;
cz_des = c_hat_vz_cmd;
cz_meas = c_hat_vz_act;
fx_des = cmd_force;
fx_meas = c_hat_fx_act;

f2 = figure('Name','Panel 2 - cy/cz/Fx + normal-angle / wind','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.05 0.05 0.90 0.88]);
tl2 = tiledlayout(f2, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_panel2_row1 = gobjects(0);

panel2_colors.des = [0.85 0.325 0.098];
panel2_colors.meas = [0.00 0.45 0.74];
panel2_colors.wind_x = [0.20 0.50 0.95];
panel2_colors.wind_y = [0.15 0.60 0.30];
panel2_colors.wind_z = [0.90 0.30 0.18];

panel2_row1_data = {
    {cy_des, cy_meas, 'cy', 'vel [m/s]'}
    {cz_des, cz_meas, 'cz', 'vel [m/s]'}
    {fx_des, fx_meas, 'Fx', 'force [N]'}
};
for i = 1:3
    ax = nexttile(tl2, i); ax_panel2_row1(end+1) = ax; %#ok<SAGROW>
    plot(time, panel2_row1_data{i}{1}, '--', 'LineWidth', 1.6, 'Color', panel2_colors.des); hold on;
    plot(time, panel2_row1_data{i}{2}, '-', 'LineWidth', lw_main, 'Color', panel2_colors.meas);
    grid on;
    apply_user_ylim(ax, ylim_panel2_row1, i, [panel2_row1_data{i}{1}; panel2_row1_data{i}{2}]);
    ylabel(panel2_row1_data{i}{4});
    title(panel2_row1_data{i}{3});
    legend({'desired', 'meas'}, 'Location', 'best');
    xlabel('time [s]');
end
apply_panel_xlim(ax_panel2_row1, xlim_panel2_row1, 'panel2_row1_xlink');

ax_panel2_angle = nexttile(tl2, 4, [1 2]);
plot(time, angle_n_alg_deg, '-', 'LineWidth', lw_main, 'Color', [0.15 0.15 0.15]); hold on;
plot(time, angle_n_geo_deg, '-', 'LineWidth', lw_main, 'Color', [0.20 0.50 0.95]);
plot(time, angle_n_f_deg, '-', 'LineWidth', lw_main, 'Color', [0.90 0.30 0.18]);
grid on;
ylim(ax_panel2_angle, ylim_panel2_angle);
xlim(ax_panel2_angle, xlim_panel2_row2);
ylabel('angle [deg]');
title('Normal Vector Estimation Angles');
legend({'cos^{-1}(\hat{n} \cdot n_{alg})', 'cos^{-1}(\hat{n} \cdot n_{geo})', 'cos^{-1}(\hat{n} \cdot n_f)'}, ...
    'Location', 'best', 'Interpreter', 'tex');
xlabel('time [s]');
if ~normal_force_panel_valid
    text(ax_panel2_angle, mean(xlim_panel2_row2), mean(ylim_panel2_angle), ...
        'normal-vector panel data are not logged in current CSV', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', [0.5 0.1 0.1]);
end

ax_panel2_wind = nexttile(tl2, 6);
plot(time, wind_force(:,1), '-', 'LineWidth', lw_wind, 'Color', panel2_colors.wind_x); hold on;
plot(time, wind_force(:,2), '-', 'LineWidth', lw_wind, 'Color', panel2_colors.wind_y);
plot(time, wind_force(:,3), '-', 'LineWidth', lw_wind, 'Color', panel2_colors.wind_z);
grid on;
apply_user_ylim(ax_panel2_wind, repmat(ylim_panel2_wind, 1, 1), 1, [wind_force(:,1); wind_force(:,2); wind_force(:,3)]);
xlim(ax_panel2_wind, xlim_panel2_row2);
ylabel('wind [N]');
title('Wind disturbance x/y/z');
legend({'x', 'y', 'z'}, 'Location', 'best');
xlabel('time [s]');
set(findall(f2, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Panel 3) Consistency compare dashboard
xlim_panel3_row1 = [15 35];
xlim_panel3_row2 = [15 35];
ylim_panel3_row1 = repmat([-0.04 0.04], 3, 1);
ylim_panel3_row2 = repmat([-0.005 0.005], 3, 1);

f3 = figure('Name','Panel 3 - Consistency compare','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.05 0.05 0.90 0.88]);
tl3 = tiledlayout(f3, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_panel3_row1 = gobjects(0);
ax_panel3_row2 = gobjects(0);

panel3_colors.mob2 = [0.20 0.50 0.95];
panel3_colors.mobc = [0.90 0.30 0.18];
panel3_colors.wind = [0.15 0.60 0.30];
panel3_colors.tauhat = [0.45 0.20 0.75];
panel3_colors.rxf_pure = [0.20 0.50 0.95];
panel3_colors.rxf_consistency = [0.90 0.30 0.18];

for i = 1:3
    ax = nexttile(tl3, i); ax_panel3_row1(end+1) = ax; %#ok<SAGROW>
    plot(time, wind_force(:,i), '-', 'LineWidth', lw_wind, 'Color', panel3_colors.wind); hold on;
    plot(time, mob2_force(:,i), '-', 'LineWidth', lw_main, 'Color', panel3_colors.mob2);
    plot(time, mob3_force(:,i), '-', 'LineWidth', lw_main, 'Color', panel3_colors.mobc);
    grid on;
    apply_user_ylim(ax, ylim_panel3_row1, i, [mob2_force(:,i); mob3_force(:,i); wind_force(:,i)]);
    ylabel('force [N]');
    title(sprintf('Force %s', axis_names{i}));
    if i == 1
        legend({'Wind external', 'MOB 2nd order', 'Consistency residual'}, 'Location', 'best');
    end
    xlabel('time [s]');

    ax = nexttile(tl3, 3 + i); ax_panel3_row2(end+1) = ax; %#ok<SAGROW>
    plot(time, mob_tau_tauhat(:,i), '-', 'LineWidth', lw_main, 'Color', panel3_colors.tauhat); hold on;
    plot(time, rxf_pure(:,i), '-', 'LineWidth', lw_main, 'Color', panel3_colors.rxf_pure);
    plot(time, rxf_consistency(:,i), '-', 'LineWidth', lw_main, 'Color', panel3_colors.rxf_consistency);
    grid on;
    apply_user_ylim(ax, ylim_panel3_row2, i, [mob_tau_tauhat(:,i); rxf_pure(:,i); rxf_consistency(:,i)]);
    ylabel('torque [Nm]');
    title(sprintf('\\tau ext / r \\times F %s', axis_names{i}));
    if i == 1
        legend({'\tau ext', 'r \times F (pure MOB F)', 'r \times F (consistency MOB F)'}, ...
            'Location', 'best', 'Interpreter', 'tex');
    end
    xlabel('time [s]');
end

apply_panel_xlim(ax_panel3_row1, xlim_panel3_row1, 'panel3_row1_xlink');
apply_panel_xlim(ax_panel3_row2, xlim_panel3_row2, 'panel3_row2_xlink');
set(findall(f3, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Panel 4) MOB force + attitude
xlim_panel4_row1 = [15 35];
xlim_panel4_row2 = [15 35];
ylim_panel4_row1 = repmat([-0.04 0.04], 3, 1);
ylim_panel4_row2 = repmat([-0.3 0.3], 3, 1);

f4 = figure('Name','Panel 4 - MOB force + attitude','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.05 0.05 0.90 0.88]);
tl4 = tiledlayout(f4, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_panel4_row1 = gobjects(0);
ax_panel4_row2 = gobjects(0);

panel4_colors.mob2 = [0.20 0.50 0.95];
panel4_colors.wind = [0.15 0.60 0.30];
panel4_colors.att_meas = [0.00 0.45 0.74];
panel4_colors.att_cmd = [0.85 0.325 0.098];

for i = 1:3
    ax = nexttile(tl4, i); ax_panel4_row1(end+1) = ax; %#ok<SAGROW>
    plot(time, wind_force(:,i), '-', 'LineWidth', lw_wind, 'Color', panel4_colors.wind); hold on;
    plot(time, mob2_force(:,i), '-', 'LineWidth', lw_main, 'Color', panel4_colors.mob2);
    grid on;
    apply_user_ylim(ax, ylim_panel4_row1, i, [mob2_force(:,i); wind_force(:,i)]);
    ylabel('force [N]');
    title(sprintf('Force %s', axis_names{i}));
    if i == 1
        legend({'Wind external', 'MOB 2nd order'}, 'Location', 'best');
    end
    xlabel('time [s]');

    ax = nexttile(tl4, 3 + i); ax_panel4_row2(end+1) = ax; %#ok<SAGROW>
    plot(time, att_pose(:,i), '-', 'LineWidth', lw_main, 'Color', panel4_colors.att_meas); hold on;
    plot(time, att_des_u(:,i), '--', 'LineWidth', 1.6, 'Color', panel4_colors.att_cmd);
    grid on;
    apply_user_ylim(ax, ylim_panel4_row2, i, [att_pose(:,i); att_des_u(:,i)]);
    ylabel(sprintf('%s [rad]', att_names{i}));
    title(sprintf('Attitude %s', att_names{i}));
    if i == 1
        legend({'measured', 'cmd'}, 'Location', 'best');
    end
    xlabel('time [s]');
end

apply_panel_xlim(ax_panel4_row1, xlim_panel4_row1, 'panel4_row1_xlink');
apply_panel_xlim(ax_panel4_row2, xlim_panel4_row2, 'panel4_row2_xlink');
set(findall(f4, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

function [force_world, rotated_vec] = thrust_world_from_pose_and_fz(roll, pitch, yaw, fz, body_vec)
    n = numel(fz);
    force_world = zeros(n, 3);
    rotated_vec = zeros(n, 3);
    if nargin < 5
        body_vec = [0, 0, 0];
    end
    for k = 1:n
        cr = cos(roll(k)); sr = sin(roll(k));
        cp = cos(pitch(k)); sp = sin(pitch(k));
        cy = cos(yaw(k)); sy = sin(yaw(k));
        r_bw = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
                sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
                -sp,   cp*sr,            cp*cr];
        force_world(k, :) = (r_bw * [0; 0; fz(k)]).';
        rotated_vec(k, :) = (r_bw * body_vec(:)).';
    end
end

function c = cross_rows(a, b)
    c = [ ...
        a(:,2).*b(:,3) - a(:,3).*b(:,2), ...
        a(:,3).*b(:,1) - a(:,1).*b(:,3), ...
        a(:,1).*b(:,2) - a(:,2).*b(:,1)];
end

function vhat = normalize_rows(v)
    vhat = nan(size(v));
    for k = 1:size(v, 1)
        n = norm(v(k, :));
        if isfinite(n) && n > 1e-12
            vhat(k, :) = v(k, :) / n;
        end
    end
end

function proj = project_rows_with_ws(force_rows, ws_rows, use_identity_on_invalid)
    if nargin < 3
        use_identity_on_invalid = false;
    end
    proj = nan(size(force_rows));
    for k = 1:size(force_rows, 1)
        f = force_rows(k, :);
        w = ws_rows(k, :);
        if any(~isfinite(f))
            continue;
        end
        if any(~isfinite(w))
            if use_identity_on_invalid
                proj(k, :) = f;
            end
            continue;
        end
        nw = norm(w);
        if nw < 1e-12
            if use_identity_on_invalid
                proj(k, :) = f;
            end
            continue;
        end
        wn = w / nw;
        proj(k, :) = f - dot(f, wn) * wn;
    end
end

function proj = project_rows_with_ws_gamma(force_rows, ws_rows, gamma_v)
    proj = nan(size(force_rows));
    for k = 1:size(force_rows, 1)
        f = force_rows(k, :);
        w = ws_rows(k, :);
        g = gamma_v(k);
        if any(~isfinite(f)) || ~isfinite(g)
            continue;
        end
        if any(~isfinite(w))
            proj(k, :) = f;
            continue;
        end
        nw = norm(w);
        if nw < 1e-12
            proj(k, :) = f;
            continue;
        end
        wn = w / nw;
        proj(k, :) = f - g * dot(f, wn) * wn;
    end
end

function ang_deg = angle_deg_rows(a, b)
    ang_deg = nan(size(a, 1), 1);
    for k = 1:size(a, 1)
        if any(~isfinite(a(k, :))) || any(~isfinite(b(k, :)))
            continue;
        end
        na = norm(a(k, :));
        nb = norm(b(k, :));
        if na < 1e-12 || nb < 1e-12
            continue;
        end
        c = dot(a(k, :), b(k, :)) / (na * nb);
        c = max(-1.0, min(1.0, c));
        ang_deg(k) = acosd(c);
    end
end

function n_gt = estimate_gt_normal(ee_pos, environment_type, wall_x, cylinder_center)
    n_gt = nan(size(ee_pos));
    for k = 1:size(ee_pos, 1)
        if any(~isfinite(ee_pos(k, :)))
            continue;
        end
        if strcmpi(environment_type, "wall")
            if ee_pos(k, 1) <= wall_x
                n_gt(k, :) = [1.0, 0.0, 0.0];
            else
                n_gt(k, :) = [-1.0, 0.0, 0.0];
            end
        else
            radial = [ee_pos(k, 1) - cylinder_center(1), ee_pos(k, 2) - cylinder_center(2), 0.0];
            nr = norm(radial);
            if nr > 1e-12
                n_gt(k, :) = -radial / nr;
            end
        end
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

function apply_yaxis_display_scale(ax_list, scale)
    for i = 1:numel(ax_list)
        ax = ax_list(i);
        ticks = yticks(ax);
        scaled_labels = arrayfun(@(v) sprintf('%.3g', scale * v), ticks, 'UniformOutput', false);
        yticklabels(ax, scaled_labels);
    end
end

function plot_cmd_meas_triplet_panel(parent_panel, time, cmd_data, meas_data, names, label_cmd, color_cmd, label_meas, color_meas, panel_title, unit_suffix, ylims, xlims, lw_cmd, lw_meas)
    tl = tiledlayout(parent_panel, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax_list = gobjects(0);
    for i = 1:3
        ax = nexttile(tl, i); ax_list(end+1) = ax; %#ok<AGROW>
        h_cmd = plot(ax, time, cmd_data(:, i), '--', 'LineWidth', lw_cmd, 'Color', color_cmd); hold(ax, 'on');
        h_meas = plot(ax, time, meas_data(:, i), '-', 'LineWidth', lw_meas, 'Color', color_meas);
        grid(ax, 'on');
        ylabel(ax, sprintf('%s %s', names{i}, unit_suffix));
        if i == 1
            title(ax, panel_title);
            legend(ax, [h_cmd h_meas], label_cmd, label_meas, 'Location', 'best');
        end
        apply_user_ylim(ax, ylims, i, [cmd_data(:, i); meas_data(:, i)]);
        if i < 3
            ax.XTickLabel = [];
        else
            xlabel(ax, 'time [s]');
        end
    end
    linkaxes(ax_list, 'x');
    if numel(xlims) == 2 && all(isfinite(xlims))
        xlim(ax_list, xlims);
    end
end

function plot_single_triplet_panel(parent_panel, time, data, names, line_color, panel_title, unit_suffix, ylims, xlims, lw)
    tl = tiledlayout(parent_panel, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile(tl, i);
        plot(time, data(:, i), '-', 'LineWidth', lw, 'Color', line_color); hold(ax, 'on');
        grid(ax, 'on');
        apply_user_ylim(ax, ylims, i, data(:, i));
        if numel(xlims) == 2 && all(isfinite(xlims))
            xlim(ax, xlims);
        end
        ylabel(ax, sprintf('%s %s', names{i}, unit_suffix));
        if i == 1
            title(ax, panel_title);
        end
        if i == 3
            xlabel(ax, 'time [s]');
        end
    end
end

function plot_three_scalar_panel_solid(parent_panel, time, data_a, data_b, data_c, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, ylims, xlims)
    ax = axes(parent_panel);
    h1 = plot(ax, time, data_a, '-', 'LineWidth', 1.25, 'Color', color_a); hold(ax, 'on');
    h2 = plot(ax, time, data_b, '-', 'LineWidth', 1.15, 'Color', color_b);
    h3 = plot(ax, time, data_c, '-', 'LineWidth', 1.15, 'Color', color_c);
    grid(ax, 'on');
    title(ax, panel_title);
    ylabel(ax, unit_suffix);
    xlabel(ax, 'time [s]');
    legend(ax, [h1 h2 h3], label_a, label_b, label_c, 'Location', 'best');
    if numel(ylims) == 2 && all(isfinite(ylims))
        ylim(ax, ylims);
    end
    if numel(xlims) == 2 && all(isfinite(xlims))
        xlim(ax, xlims);
    end
end

function plot_three_scalar_panel_solid_or_message(parent_panel, time, data_a, data_b, data_c, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, ylims, xlims, is_available, message_text)
    if is_available
        plot_three_scalar_panel_solid(parent_panel, time, data_a, data_b, data_c, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, ylims, xlims);
        return;
    end
    ax = axes(parent_panel);
    axis(ax, 'off');
    title(ax, panel_title);
    text(ax, 0.5, 0.5, message_text, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'Color', [0.45 0.10 0.10], 'FontSize', 10);
end

function plot_three_triplet_panel_solid(parent_panel, time, data_a, data_b, data_c, names, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, lw_a, lw_b, lw_c, ylims, xlims)
    tl = tiledlayout(parent_panel, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax_list = gobjects(0);
    for i = 1:3
        ax = nexttile(tl, i); ax_list(end+1) = ax; %#ok<AGROW>
        h1 = plot(ax, time, data_a(:, i), '-', 'LineWidth', lw_a, 'Color', color_a); hold(ax, 'on');
        h2 = plot(ax, time, data_b(:, i), '-', 'LineWidth', lw_b, 'Color', color_b);
        h3 = plot(ax, time, data_c(:, i), '-', 'LineWidth', lw_c, 'Color', color_c);
        grid(ax, 'on');
        ylabel(ax, sprintf('%s %s', names{i}, unit_suffix));
        if i == 1
            title(ax, panel_title);
            legend(ax, [h1 h2 h3], label_a, label_b, label_c, 'Location', 'best');
        end
        apply_user_ylim(ax, ylims, i, [data_a(:, i); data_b(:, i); data_c(:, i)]);
        if i < 3
            ax.XTickLabel = [];
        else
            xlabel(ax, 'time [s]');
        end
    end
    linkaxes(ax_list, 'x');
    if numel(xlims) == 2 && all(isfinite(xlims))
        xlim(ax_list, xlims);
    end
end

function plot_three_triplet_panel_solid_or_message(parent_panel, time, data_a, data_b, data_c, names, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, lw_a, lw_b, lw_c, ylims, xlims, is_available, message_text)
    if is_available
        plot_three_triplet_panel_solid(parent_panel, time, data_a, data_b, data_c, names, label_a, color_a, label_b, color_b, label_c, color_c, panel_title, unit_suffix, lw_a, lw_b, lw_c, ylims, xlims);
        return;
    end
    ax = axes(parent_panel);
    axis(ax, 'off');
    title(ax, panel_title);
    text(ax, 0.5, 0.5, message_text, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'Color', [0.45 0.10 0.10], 'FontSize', 10);
end

function data_filt = lowpass_triplet_by_time(time, data, cutoff_hz)
    data_filt = data;
    if ~(isfinite(cutoff_hz) && cutoff_hz > 0.0)
        return;
    end

    for i = 2:size(data, 1)
        dt = time(i) - time(i - 1);
        if ~(isfinite(dt) && dt > 0.0)
            data_filt(i, :) = data_filt(i - 1, :);
            continue;
        end
        tau = 1.0 / (2.0 * pi * cutoff_hz);
        alpha = max(0.0, min(1.0, dt / (tau + dt)));
        for j = 1:size(data, 2)
            x = data(i, j);
            y_prev = data_filt(i - 1, j);
            if ~isfinite(x)
                data_filt(i, j) = y_prev;
            elseif ~isfinite(y_prev)
                data_filt(i, j) = x;
            else
                data_filt(i, j) = y_prev + alpha * (x - y_prev);
            end
        end
    end
end

function plot_compare_triplet_panel(parent_panel, time, data_a, data_b, names, label_a, color_a, label_b, color_b, panel_title, unit_suffix, lw_a, lw_b, ylims, xlims)
    tl = tiledlayout(parent_panel, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax_list = gobjects(0);
    for i = 1:3
        ax = nexttile(tl, i); ax_list(end+1) = ax; %#ok<AGROW>
        ha = plot(ax, time, data_a(:, i), '-', 'LineWidth', lw_a, 'Color', color_a); hold(ax, 'on');
        hb = plot(ax, time, data_b(:, i), '-', 'LineWidth', lw_b, 'Color', color_b);
        grid(ax, 'on');
        ylabel(ax, sprintf('%s %s', names{i}, unit_suffix));
        apply_user_ylim(ax, ylims, i, [data_a(:, i); data_b(:, i)]);
        if i == 1
            title(ax, panel_title);
            legend(ax, [ha hb], label_a, label_b, 'Location', 'best');
        end
        if i < 3
            ax.XTickLabel = [];
        else
            xlabel(ax, 'time [s]');
        end
    end
    linkaxes(ax_list, 'x');
    if numel(xlims) == 2 && all(isfinite(xlims))
        xlim(ax_list, xlims);
    end
end

function plot_compare_triplet_or_message(parent_panel, time, data_a, data_b, names, label_a, color_a, label_b, color_b, panel_title, unit_suffix, lw_a, lw_b, ylims, xlims, is_available, message_text)
    if is_available
        plot_compare_triplet_panel(parent_panel, time, data_a, data_b, names, label_a, color_a, label_b, color_b, panel_title, unit_suffix, lw_a, lw_b, ylims, xlims);
        return;
    end
    ax = axes(parent_panel);
    axis(ax, 'off');
    title(ax, panel_title);
    text(ax, 0.5, 0.5, message_text, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'Color', [0.45 0.10 0.10], 'FontSize', 10);
end

function plot_scalar_or_message(parent_panel, time, data, color, panel_title, unit_suffix, ylims, xlims, is_available, message_text)
    if is_available
        ax = axes(parent_panel);
        plot(ax, time, data, '-', 'LineWidth', 1.25, 'Color', color);
        grid(ax, 'on');
        title(ax, panel_title);
        ylabel(ax, unit_suffix);
        xlabel(ax, 'time [s]');
        if numel(ylims) == 2 && all(isfinite(ylims))
            ylim(ax, ylims);
        end
        if numel(xlims) == 2 && all(isfinite(xlims))
            xlim(ax, xlims);
        end
        return;
    end
    ax = axes(parent_panel);
    axis(ax, 'off');
    title(ax, panel_title);
    text(ax, 0.5, 0.5, message_text, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'Color', [0.45 0.10 0.10], 'FontSize', 10);
end

function v = local_get_any(T, vars, names)
    v = nan(height(T), 1);
    for idx = 1:numel(names)
        name = string(names(idx));
        if any(vars == name)
            v = T{:, char(name)};
            return;
        end
    end
end
