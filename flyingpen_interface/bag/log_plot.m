%% plot_flyingpen_log.m
% Reads data_logger CSV and compares desired vs actual signals with subplots.
% Column order exactly as in Float64MultiArray (0..45):
% 0  t_sec
% 1  cmd_x, 2 cmd_y, 3 cmd_z, 4 cmd_yaw
% 5  pos_x, 6 pos_y, 7 pos_z
% 8  roll, 9 pitch, 10 yaw
% 11 vel_x, 12 vel_y, 13 vel_z
% 14 w_x, 15 w_y, 16 w_z
% 17 acc_x, 18 acc_y, 19 acc_z
% 20 angacc_x, 21 angacc_y, 22 angacc_z
% 23 vdes_x, 24 vdes_y, 25 vdes_z
% 26 rolld, 27 pitchd, 28 yawd
% 29 wdes_x, 30 wdes_y, 31 wdes_z
% 32 tau_x, 33 tau_y, 34 tau_z, 35 Fz
% 36 contact_Fx_raw, 37 contact_Fy_raw, 38 contact_Fz_raw
% 39 contact_Fx_filt,40 contact_Fy_filt,41 contact_Fz_filt
% 42 cmd_force
% 43 F_error_dot_raw, 44 F_error_dot_filt
% 45 validity_bitmask

clear; clc;

%% ---- pick CSV ----
defaultDir = fullfile(getenv("HOME"), "mujoco_crazyflie", "src", "flyingpen_interface", "bag");
if ~isfolder(defaultDir)
    defaultDir = pwd;
end

[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select data_logger CSV");
if isequal(file,0)
    disp("Canceled.");
    return;
end
csvPath = fullfile(path, file);

%% ---- read ----
opts = detectImportOptions(csvPath);
opts = setvartype(opts, 'double');   % ensure numeric
T = readtable(csvPath, opts);

A = table2array(T);
if size(A,2) < 46
    error("CSV must have >= 46 columns, got %d", size(A,2));
end

t = A(:,1);
t = t - t(1); % start at 0

% Helper to grab columns by 1-based index in MATLAB:
col = @(k) A(:,k);

% --- Commands (pos_cmd) ---
cmd_x   = col(2); cmd_y = col(3); cmd_z = col(4); cmd_yaw = col(5);

% --- Actual pose ---
pos_x   = col(6); pos_y = col(7); pos_z = col(8);
roll    = col(9); pitch = col(10); yaw   = col(11);

% --- Actual velocities ---
vel_x = col(12); vel_y = col(13); vel_z = col(14);
w_x   = col(15); w_y   = col(16); w_z   = col(17);

% --- Actual acc ---
acc_x = col(18); acc_y = col(19); acc_z = col(20);
angacc_x = col(21); angacc_y = col(22); angacc_z = col(23);

% --- Desired ---
vdes_x = col(24); vdes_y = col(25); vdes_z = col(26);
rolld  = col(27); pitchd = col(28); yawd  = col(29);
wdes_x = col(30); wdes_y = col(31); wdes_z = col(32);

% --- Input (controller output) ---
tau_x = col(33); tau_y = col(34); tau_z = col(35); Fz = col(36);

% --- Contact Force ---
contact_Fx_raw = col(37); contact_Fy_raw = col(38); contact_Fz_raw = col(39);

% --- Contact Force filt ---
contact_Fx_filt = col(40); contact_Fy_filt = col(41); contact_Fz_filt = col(42);

% --- Command Force ---
cmd_F = col(43);

lpf_force_raw = col(44); lpf_force_filter = col(45);

mask = uint32(col(46));

%% ---- convenience: unwrap yaw for nicer plots ----
yaw_u    = unwrap(yaw);
cmd_yaw_u = unwrap(cmd_yaw);
yawd_u   = unwrap(yawd);

%% ---- Figure 1: Position commands vs actual position ----
figure('Name','Position: cmd vs actual','Color','w');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, cmd_x, 'LineWidth', 1.2); hold on;
plot(t, pos_x, 'LineWidth', 1.2);
grid on; ylabel('x [m]'); legend('cmd x','pos x');

nexttile;
plot(t, cmd_y, 'LineWidth', 1.2); hold on;
plot(t, pos_y, 'LineWidth', 1.2);
grid on; ylabel('y [m]'); legend('cmd y','pos y');

nexttile;
plot(t, cmd_z, 'LineWidth', 1.2); hold on;
plot(t, pos_z, 'LineWidth', 1.2);
grid on; ylabel('z [m]'); xlabel('t [s]'); legend('cmd z','pos z');

%% ---- Figure 2: Desired velocity vs actual velocity ----
figure('Name','Velocity: v_des vs vel','Color','w');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, vdes_x, 'LineWidth', 1.2); hold on;
plot(t, vel_x,  'LineWidth', 1.2);
grid on; ylabel('vx [m/s]'); legend('v\_des x','vel x');

nexttile;
plot(t, vdes_y, 'LineWidth', 1.2); hold on;
plot(t, vel_y,  'LineWidth', 1.2);
grid on; ylabel('vy [m/s]'); legend('v\_des y','vel y');

nexttile;
plot(t, vdes_z, 'LineWidth', 1.2); hold on;
plot(t, vel_z,  'LineWidth', 1.2);
grid on; ylabel('vz [m/s]'); xlabel('t [s]'); legend('v\_des z','vel z');

%% ---- Figure 3: Desired RPY vs actual RPY ----
figure('Name','Attitude: rpy_des vs rpy','Color','w');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, rolld, 'LineWidth', 1.2); hold on;
plot(t, roll,  'LineWidth', 1.2);
grid on; ylabel('roll [rad]'); legend('roll\_des','roll');

nexttile;
plot(t, pitchd, 'LineWidth', 1.2); hold on;
plot(t, pitch,  'LineWidth', 1.2);
grid on; ylabel('pitch [rad]'); legend('pitch\_des','pitch');

nexttile;
plot(t, yawd_u, 'LineWidth', 1.2); hold on;
plot(t, yaw_u,  'LineWidth', 1.2); hold on;
plot(t, cmd_yaw_u, '--', 'LineWidth', 1.0);
grid on; ylabel('yaw [rad]'); xlabel('t [s]');
legend('yaw\_des','yaw','cmd yaw');

%% ---- Figure 4: Desired body rates vs actual body rates ----
figure('Name','Body rate: w_des vs w','Color','w');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, wdes_x, 'LineWidth', 1.2); hold on;
plot(t, w_x,    'LineWidth', 1.2);
plot(t, tau_x * 100, 'LineWidth', 1.2); grid on;
grid on; ylabel('p [rad/s]'); legend('w\_des x','w x','\tau_x');

nexttile;
plot(t, wdes_y, 'LineWidth', 1.2); hold on;
plot(t, w_y,    'LineWidth', 1.2);
grid on; ylabel('q [rad/s]'); legend('w\_des y','w y');

nexttile;
plot(t, wdes_z, 'LineWidth', 1.2); hold on;
plot(t, w_z,    'LineWidth', 1.2);
grid on; ylabel('r [rad/s]'); xlabel('t [s]'); legend('w\_des z','w z');

%% ---- Figure 5: Control outputs ----
figure('Name','Control outputs (tau, Fz)','Color','w');
tiledlayout(4,1,'Padding','compact','TileSpacing','compact');

nexttile; plot(t, tau_x, 'LineWidth', 1.2); grid on; ylabel('\tau_x');
nexttile; plot(t, tau_y, 'LineWidth', 1.2); grid on; ylabel('\tau_y');
nexttile; plot(t, tau_z, 'LineWidth', 1.2); grid on; ylabel('\tau_z');
nexttile; plot(t, Fz,    'LineWidth', 1.2); grid on; ylabel('Fz'); xlabel('t [s]');

%% ---- Figure 6: Contact force raw ----
figure('Name','Contact force raw: F_contact_raw (world)','Color','w');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, contact_Fx_raw, 'LineWidth', 1.2);
grid on; ylabel('F_{c,x} [N]'); legend('contact F_x_raw');
ylim([-0.05 0.25])

nexttile;
plot(t, contact_Fy_raw, 'LineWidth', 1.2);
grid on; ylabel('F_{c,y} [N]'); legend('contact F_y_raw');
ylim([-0.05 0.25])

nexttile;
plot(t, contact_Fz_raw, 'LineWidth', 1.2);
grid on; ylabel('F_{c,z} [N]'); xlabel('t [s]'); legend('contact F_z_raw');
ylim([-0.05 0.25])


%% ---- Figure 7: Contact force filter ----
figure('Name','Contact force filter: F_contact_filt (world)','Color','w');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, contact_Fx_filt, 'LineWidth', 1.2);
grid on; ylabel('F_{c,x} [N]'); legend('contact F_x_filt');
ylim([-0.05 0.25])

nexttile;
plot(t, contact_Fy_filt, 'LineWidth', 1.2);
grid on; ylabel('F_{c,y} [N]'); legend('contact F_y_filt');
ylim([-0.05 0.25])

nexttile;
plot(t, contact_Fz_filt, 'LineWidth', 1.2);
grid on; ylabel('F_{c,z} [N]'); xlabel('t [s]'); legend('contact F_z_filt');
ylim([-0.05 0.25])


%% ---- Figure 8: Fx raw vs filtered (overlay) ----
figure('Name','Contact Fx: raw vs filtered','Color','w');

plot(t, contact_Fx_raw,  'LineWidth', 2); hold on;
plot(t, contact_Fx_filt, 'LineWidth', 2);
plot(t, cmd_F, 'LineWidth', 2);
grid on;
%xlim([90 100])
xlabel('t [s]');
ylabel('F_{c,x} [N]');
legend('contact F_x raw','contact F_x filt', 'cmd Force', 'Location','best');


%% ---- Figure 9: Command force ----c
figure('Name','Command Force : F_cmd','Color','w');
tiledlayout(1,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, cmd_F, 'LineWidth', 1.2);
grid on; ylabel('F_{cmd} [N]'); xlabel('t [s]'); legend('F_{cmd}');
ylim([-0.08 0.08])

%% ---- Figure 10: LPF Force error dot----
figure('Name','LPF debug: F\_error\_dot','Color','w');

plot(t, lpf_force_raw, 'LineWidth', 1.2); hold on;
plot(t, lpf_force_filter, 'LineWidth', 1.2);
grid on;

xlabel('t [s]');
ylabel('F\_error\_dot [N/s]');
legend('raw','filtered');


%% ---- (Optional) Validity bitmask visualization ----
figure('Name','Validity bitmask','Color','w');
plot(t, double(mask), 'LineWidth', 1.2);
grid on; xlabel('t [s]'); ylabel('mask (uint32 as double)');
title('Validity bitmask (see data_logger definition)');

disp("Done. Plots generated:");
disp("1) Position cmd vs pos");
disp("2) v_des vs vel");
disp("3) rpy_des vs rpy (+ cmd yaw)");
disp("4) w_des vs w");
disp("5) tau/Fz");
disp("6) validity mask");
