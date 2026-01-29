#!/usr/bin/env python3
import os
import time
import signal
import threading
from typing import Optional, Sequence, Union

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import mujoco
import mujoco.viewer

from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from std_msgs.msg import Float32MultiArray

# -------------------- rates --------------------
PHYSICS_HZ = 1000.0          # MuJoCo integration rate
PUB_HZ = 400.0               # ROS publish rate (decimated from physics)
VIEWER_HZ = 60.0             # viewer sync rate (keep low to avoid blocking sim)


# -------------------- math utils --------------------
def quat_wxyz_to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
    # MuJoCo free joint quaternion: (w, x, y, z) -> ROS: (x, y, z, w)
    w, x, y, z = q_wxyz
    return np.array([x, y, z, w], dtype=float)


def rotmat_from_quat_wxyz(q: np.ndarray) -> np.ndarray:
    # q=(w,x,y,z), returns R: body->world
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=float)


def quat_normalize_wxyz(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_mul_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product. Both are (w,x,y,z). Returns (w,x,y,z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=float)


def rotvec_to_quat_wxyz(r: np.ndarray) -> np.ndarray:
    """Rotation vector r (rad) -> quaternion (w,x,y,z)."""
    angle = float(np.linalg.norm(r))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    axis = r / angle
    half = 0.5 * angle
    s = np.sin(half)
    return np.array([np.cos(half), axis[0]*s, axis[1]*s, axis[2]*s], dtype=float)


# -------------------- Node --------------------
class CrazyfliePlant(Node):
    """
    Sub:
      /crazyflie/in/input : Float32MultiArray [tau_x, tau_y, tau_z, Fz] (body frame)

    Pub:
      /crazyflie/out/pose        : PoseStamped (world, quat)                  <-- noisy pose (optional)
      /crazyflie/out/vel         : Vector3Stamped (world linear velocity)     <-- noisy vel (optional)
      /crazyflie/out/ang_vel     : Vector3Stamped (body angular velocity)     <-- gyro + optional extra noise
      /crazyflie/out/acc         : Vector3Stamped (world linear acceleration)
      /crazyflie/out/ang_acc     : Vector3Stamped (body angular acceleration) <-- diff of gyro + optional noise
      /crazyflie/out/ang_vel_gt  : Vector3Stamped (body angular velocity)     <-- from qvel (GT)
    """

    def __init__(self):
        super().__init__("mujoco_crazyflie_plant")

        # ---- params ----
        self.declare_parameter("physics_hz", PHYSICS_HZ)
        self.declare_parameter("pub_hz", PUB_HZ)
        self.declare_parameter("viewer_hz", VIEWER_HZ)

        self.physics_hz = float(self.get_parameter("physics_hz").value)
        self.pub_hz = float(self.get_parameter("pub_hz").value)
        self.viewer_hz = float(self.get_parameter("viewer_hz").value)

        # Mixer params
        self.declare_parameter("arm_xy", 0.035355)  # meters
        self.declare_parameter("k_tau", 0.00594)    # yaw reaction torque per thrust [N·m / N]
        self.declare_parameter("motor_dir", [1.0, -1.0, 1.0, -1.0])  # +1/-1 spin direction
        self.declare_parameter("thrust_min", 0.0)
        self.declare_parameter("thrust_max", 0.20)

        self.a = float(self.get_parameter("arm_xy").value)
        self.k_tau = float(self.get_parameter("k_tau").value)
        self.motor_dir = np.array(self.get_parameter("motor_dir").value, dtype=float)
        self.thrust_min = float(self.get_parameter("thrust_min").value)
        self.thrust_max = float(self.get_parameter("thrust_max").value)
        if self.motor_dir.shape[0] != 4:
            self.motor_dir = np.array([1.0, -1.0, 1.0, -1.0], dtype=float)

        # ---- NOISE params (read from plant: ros__parameters: noise: ...) ----
        # variance (σ²) units:
        #   pos_var      : m^2
        #   vel_var      : (m/s)^2
        #   att_var      : rad^2  (small-angle rotvec, body axes)
        #   ang_vel_var  : (rad/s)^2
        #   ang_acc_var  : (rad/s^2)^2
        self.declare_parameter("noise.enable", True)
        self.declare_parameter("noise.seed", 0)  # 0 => random seed from entropy
        self.declare_parameter("noise.pos_var", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.vel_var", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.att_var", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.ang_vel_var", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.ang_acc_var", [0.0, 0.0, 0.0])

        self.noise_enable = bool(self.get_parameter("noise.enable").value)
        seed = int(self.get_parameter("noise.seed").value)
        self.rng = np.random.default_rng(None if seed == 0 else seed)

        # internal storage (std, not var)
        self._pos_std = self._var_to_std3(self.get_parameter("noise.pos_var").value)
        self._vel_std = self._var_to_std3(self.get_parameter("noise.vel_var").value)
        self._att_std = self._var_to_std3(self.get_parameter("noise.att_var").value)
        self._ang_vel_std = self._var_to_std3(self.get_parameter("noise.ang_vel_var").value)
        self._ang_acc_std = self._var_to_std3(self.get_parameter("noise.ang_acc_var").value)

        # ---- Contact arrow viz params ----
        self.declare_parameter("viz.contact_arrows.enable", True)
        self.declare_parameter("viz.contact_arrows.scale", 4.5)   # [m/N] 정도로 생각 (튜닝)
        self.declare_parameter("viz.contact_arrows.width", 0.008)   # 화살표 두께(대충)
        self.declare_parameter("viz.contact_arrows.max", 64)        # 너무 많으면 렉

        self.viz_contact_enable = bool(self.get_parameter("viz.contact_arrows.enable").value)
        self.viz_contact_scale = float(self.get_parameter("viz.contact_arrows.scale").value)
        self.viz_contact_width = float(self.get_parameter("viz.contact_arrows.width").value)
        self.viz_contact_max = int(self.get_parameter("viz.contact_arrows.max").value)

        # 6D force buffer for mj_contactForce
        self._cf6 = np.zeros(6, dtype=float)

        # ---- 수치 미분 + LPF params ----
        self.declare_parameter("contact_filter.enable", True)
        self.declare_parameter("contact_filter.cutoff_hz", 5.0)     # 4~5 Hz 추천
        self.declare_parameter("contact_filter.timer_hz", 100.0)    # 수치미분 전용 콜백 주기
        self.declare_parameter("contact_filter.use_exp_alpha", True)


        self.contact_filter_enable = bool(self.get_parameter("contact_filter.enable").value)
        self.contact_cutoff_hz = float(self.get_parameter("contact_filter.cutoff_hz").value)
        self.contact_timer_hz = float(self.get_parameter("contact_filter.timer_hz").value)
        self.use_exp_alpha = bool(self.get_parameter("contact_filter.use_exp_alpha").value)


        # raw force 최신값 버퍼 (publish_outputs에서 업데이트)
        self._F_raw_latest = np.zeros(3, dtype=float)

        # 수치미분 상태
        self._F_raw_prev = np.zeros(3, dtype=float)

        # 필터/미분 결과 상태(100Hz 타이머가 갱신)
        self._F_filt_latest = np.zeros(3, dtype=float)
        self._Fdot_raw_latest = np.zeros(3, dtype=float)
        self._Fdot_filt_latest = np.zeros(3, dtype=float)

        # 타이머 dt 실측용
        self._contact_timer_prev_ns = None


        self.timer_contact = None
        if self.contact_filter_enable:
            period = 1.0 / self.contact_timer_hz
            self.timer_contact = self.create_timer(period, self.cb_contact_diff_100hz)



        self.get_logger().info(
            "Noise(plant params): "
            f"enable={self.noise_enable}, seed={seed}, "
            f"pos_std={self._pos_std}, vel_std={self._vel_std}, att_std={self._att_std}, "
            f"ang_vel_std={self._ang_vel_std}, ang_acc_std={self._ang_acc_std}"
        )

        # ---- MuJoCo model ----
        pkg_share = get_package_share_directory("plant")
        xml_path = os.path.join(pkg_share, "data", "cf21B_500.xml")
        self.get_logger().info(f"Loading MuJoCo model: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.model.opt.timestep = 1.0 / max(1e-9, self.physics_hz)

        # ---- sensors ----
        self.imu_acc_sid = self._sensor_id("imu_acc")
        self.imu_gyro_sid = self._sensor_id("imu_gyro")

        # ---- allocation matrix (X config) ----
        a = self.a
        x = np.array([+a, -a, -a, +a], dtype=float)
        y = np.array([-a, -a, +a, +a], dtype=float)
        d = self.motor_dir
        k = self.k_tau

        self.B = np.vstack([
            y,              # tau_x
            -x,             # tau_y
            d * k,          # tau_z from reaction torque
            np.ones(4),     # Fz
        ]).astype(float)
        self.B_pinv = np.linalg.pinv(self.B)

        # ---- input ----
        self.u = np.zeros(4, dtype=float)  # [tau_x, tau_y, tau_z, Fz]

        # ---- threading ----
        self._lock = threading.Lock()
        self._stop = False

        # ---- numerical diff state (gyro used for ang_acc) ----
        self._prev_gyro_used_B: Optional[np.ndarray] = None

        # ---- actuators ----
        def _act_id(name: str) -> Optional[int]:
            try:
                return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            except Exception:
                return None

        self.act_force_ids = []
        self.act_torque_ids = []
        for i in range(4):
            self.act_force_ids.append(_act_id(f"motor{i}_force"))
            self.act_torque_ids.append(_act_id(f"motor{i}_torque"))

        if any(v is None or v < 0 for v in self.act_force_ids):
            self.get_logger().warn(
                f"Could not find all motor*_force actuators. act_force_ids={self.act_force_ids}. "
                "Fallback to ctrl[0:4]."
            )
        if any(v is None or v < 0 for v in self.act_torque_ids):
            self.get_logger().warn(
                f"Could not find all motor*_torque actuators. act_torque_ids={self.act_torque_ids}. "
                "Reaction torque won't be applied unless torque actuators exist."
            )
        else:
            self.get_logger().info(f"Found torque actuators: {self.act_torque_ids}")

        # ---- ROS IO ----
        self.sub_input = self.create_subscription(
            Float32MultiArray, "/crazyflie/in/input", self.cb_input, 10
        )

        self.pub_pose = self.create_publisher(PoseStamped, "/crazyflie/out/pose", 10)
        self.pub_vel = self.create_publisher(Vector3Stamped, "/crazyflie/out/vel", 10)
        self.pub_angvel = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel", 10)
        self.pub_acc = self.create_publisher(Vector3Stamped, "/crazyflie/out/acc", 10)
        self.pub_angacc = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_acc", 10)
        self.pub_angvel_gt = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel_gt", 10)
        self.pub_contact_force = self.create_publisher(WrenchStamped, "/crazyflie/out/EE_contact_force", 10)
        self.pub_contact_force_filt = self.create_publisher(WrenchStamped, "/crazyflie/out/EE_contact_force_filt", 10)



        # ---- threads start ----
        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.viewer_thread.start()
        self.sim_thread.start()

        self.get_logger().info(
            f"Running: physics={self.physics_hz:.1f}Hz, pub={self.pub_hz:.1f}Hz, viewer={self.viewer_hz:.1f}Hz"
        )
        self.get_logger().info("Fix applied: publish is STEP-BASED (no publish catch-up without mj_step).")
        self.get_logger().info("LPF removed: publishing pose/vel/gyro/angacc with optional noise.")

    # ------------------------ NOISE helpers ------------------------
    @staticmethod
    def _var_to_std3(v: Union[float, Sequence[float]]) -> np.ndarray:
        """Convert variance(σ²) -> std(σ) vector length 3."""
        if isinstance(v, (int, float)):
            var = np.array([float(v)] * 3, dtype=float)
        else:
            arr = np.array(list(v), dtype=float)
            if arr.size == 1:
                var = np.array([float(arr[0])] * 3, dtype=float)
            else:
                var = arr.reshape(3)
        var = np.clip(var, 0.0, None)
        return np.sqrt(var)

    def _randn3(self, std3: np.ndarray) -> np.ndarray:
        if (not self.noise_enable) or np.all(std3 <= 0.0):
            return np.zeros(3, dtype=float)
        return self.rng.normal(0.0, std3, size=3).astype(float)

    # ------------------------ helpers ------------------------
    def _sensor_id(self, name: str) -> Optional[int]:
        try:
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            self.get_logger().info(f"Sensor found: {name}")
            return sid
        except Exception:
            self.get_logger().warn(f"Sensor NOT found: {name}")
            return None

    def _read_sensor_vec3(self, sid: Optional[int]) -> np.ndarray:
        if sid is None:
            return np.zeros(3, dtype=float)
        adr = int(self.model.sensor_adr[sid])
        dim = int(self.model.sensor_dim[sid])  # expected 3
        return np.array(self.data.sensordata[adr:adr + dim], dtype=float)

    # ------------------------ sensors ------------------------
    def read_imu_acc_world(self, quat_wxyz: np.ndarray) -> np.ndarray:
        # body accel includes gravity -> world accel = R*acc_B + g_W
        acc_B = self._read_sensor_vec3(self.imu_acc_sid)
        R_BW = rotmat_from_quat_wxyz(quat_wxyz)
        g_W = np.array(self.model.opt.gravity, dtype=float)
        return R_BW @ acc_B + g_W

    def read_imu_gyro_body(self) -> np.ndarray:
        return self._read_sensor_vec3(self.imu_gyro_sid)

    # ------------------------ ROS input ------------------------
    def cb_input(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn("'/crazyflie/in/input' needs 4 floats: [tau_x, tau_y, tau_z, Fz]")
            return
        with self._lock:
            self.u[:] = np.array(msg.data[:4], dtype=float)

    # ------------------------ Control Allocation ------------------------
    def apply_control(self):
        tau_x, tau_y, tau_z, Fz = self.u

        w = np.array([tau_x, tau_y, tau_z, Fz], dtype=float)
        f = self.B_pinv @ w
        f_clip = np.clip(f, self.thrust_min, self.thrust_max)

        # apply thrust
        if all(v is not None and v >= 0 for v in self.act_force_ids):
            for i in range(4):
                self.data.ctrl[self.act_force_ids[i]] = float(f_clip[i])
        else:
            self.data.ctrl[0:4] = f_clip

        # apply reaction torque per motor
        tau_m = self.motor_dir * self.k_tau * f_clip
        if all(v is not None and v >= 0 for v in self.act_torque_ids):
            for i in range(4):
                self.data.ctrl[self.act_torque_ids[i]] = float(tau_m[i])

    # ------------------------ State read ------------------------
    def read_state(self):
        pos_W = np.array(self.data.qpos[0:3], dtype=float)
        quat_wxyz = np.array(self.data.qpos[3:7], dtype=float)

        linvel_W = np.array(self.data.qvel[0:3], dtype=float)
        angvel_W = np.array(self.data.qvel[3:6], dtype=float)

        R_BW = rotmat_from_quat_wxyz(quat_wxyz)
        angvel_B = R_BW.T @ angvel_W
        return pos_W, quat_wxyz, linvel_W, angvel_B

    # ------------------------ Publish (STEP-BASED) ------------------------
    def publish_outputs(self, dt_sim: float):
        pos_W, quat_wxyz_meas, linvel_W, angvel_B_gt = self.read_state()

        # normalize GT quat
        quat_wxyz = quat_normalize_wxyz(quat_wxyz_meas)

        # --- add noise to pos ---
        pos_W_noisy = pos_W + self._randn3(self._pos_std)

        # --- add noise to attitude (small-angle rotvec about BODY axes) ---
        dtheta_B = self._randn3(self._att_std)  # rad
        dq = rotvec_to_quat_wxyz(dtheta_B)
        quat_wxyz_noisy = quat_normalize_wxyz(quat_mul_wxyz(quat_wxyz, dq))

        # --- IMU sensors ---
        # acc: use noisy attitude for transforming (measurement chain 느낌)
        linacc_W = self.read_imu_acc_world(quat_wxyz_noisy)

        gyro_B = self.read_imu_gyro_body()              # may already include MJCF noise
        gyro_B_used = gyro_B + self._randn3(self._ang_vel_std)  # additional noise

        # --- add noise to linear velocity ---
        linvel_W_noisy = linvel_W + self._randn3(self._vel_std)

        # --- ang acc from noisy gyro (then optional extra noise) ---
        dt = max(1e-6, float(dt_sim))
        if self._prev_gyro_used_B is None:
            angacc_B = np.zeros(3, dtype=float)
        else:
            angacc_B = (gyro_B_used - self._prev_gyro_used_B) / dt
        self._prev_gyro_used_B = gyro_B_used.copy()

        angacc_B_noisy = angacc_B + self._randn3(self._ang_acc_std)

        stamp = self.get_clock().now().to_msg()

        # ---- pose ----
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(pos_W_noisy[0])
        pose_msg.pose.position.y = float(pos_W_noisy[1])
        pose_msg.pose.position.z = float(pos_W_noisy[2])
        q_xyzw = quat_wxyz_to_xyzw(quat_wxyz_noisy)
        pose_msg.pose.orientation.x = float(q_xyzw[0])
        pose_msg.pose.orientation.y = float(q_xyzw[1])
        pose_msg.pose.orientation.z = float(q_xyzw[2])
        pose_msg.pose.orientation.w = float(q_xyzw[3])
        self.pub_pose.publish(pose_msg)

        # ---- vel ----
        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "world"
        vel_msg.vector.x = float(linvel_W_noisy[0])
        vel_msg.vector.y = float(linvel_W_noisy[1])
        vel_msg.vector.z = float(linvel_W_noisy[2])
        self.pub_vel.publish(vel_msg)

        # ---- ang vel (gyro) ----
        w_msg = Vector3Stamped()
        w_msg.header.stamp = stamp
        w_msg.header.frame_id = "body"
        w_msg.vector.x = float(gyro_B_used[0])
        w_msg.vector.y = float(gyro_B_used[1])
        w_msg.vector.z = float(gyro_B_used[2])
        self.pub_angvel.publish(w_msg)

        # ---- ang vel GT ----
        wgt_msg = Vector3Stamped()
        wgt_msg.header.stamp = stamp
        wgt_msg.header.frame_id = "body"
        wgt_msg.vector.x = float(angvel_B_gt[0])
        wgt_msg.vector.y = float(angvel_B_gt[1])
        wgt_msg.vector.z = float(angvel_B_gt[2])
        self.pub_angvel_gt.publish(wgt_msg)

        # ---- acc ----
        acc_msg = Vector3Stamped()
        acc_msg.header.stamp = stamp
        acc_msg.header.frame_id = "world"
        acc_msg.vector.x = float(linacc_W[0])
        acc_msg.vector.y = float(linacc_W[1])
        acc_msg.vector.z = float(linacc_W[2])
        self.pub_acc.publish(acc_msg)

        # ---- ang acc ----
        a_msg = Vector3Stamped()
        a_msg.header.stamp = stamp
        a_msg.header.frame_id = "body"
        a_msg.vector.x = float(angacc_B_noisy[0])
        a_msg.vector.y = float(angacc_B_noisy[1])
        a_msg.vector.z = float(angacc_B_noisy[2])
        self.pub_angacc.publish(a_msg)


        # contact Force + LPF
        self.mjfc(self.model, self.data)   # self.Fw, self.rf, self.fcn 갱신
        F_raw = self.Fw.copy()             # 시작의 데이터

        # 최신 raw force 저장 (타이머가 소비)
        self._F_raw_latest = F_raw.copy()

        # ---- contact force raw publish ----
        cf_msg = WrenchStamped()
        cf_msg.header.stamp = stamp
        cf_msg.header.frame_id = "world"

        cf_msg.wrench.force.x = float(F_raw[0])
        cf_msg.wrench.force.y = float(F_raw[1])
        cf_msg.wrench.force.z = float(F_raw[2])

        # torque는 아직 미계산이면 0
        cf_msg.wrench.torque.x = 0.0
        cf_msg.wrench.torque.y = 0.0
        cf_msg.wrench.torque.z = 0.0

        self.pub_contact_force.publish(cf_msg)




    # ------------------------ threads ------------------------
    def sim_loop(self):
        dt = 1.0 / max(1e-9, self.physics_hz)
        pub_decim = max(1, int(round(self.physics_hz / max(1e-9, self.pub_hz))))

        step_count = 0
        next_step_wall = time.perf_counter()

        while rclpy.ok() and not self._stop:
            now = time.perf_counter()
            if now < next_step_wall:
                time.sleep(next_step_wall - now)
                continue

            with self._lock:
                self.apply_control()
                mujoco.mj_step(self.model, self.data)
                step_count += 1

                if (step_count % pub_decim) == 0:
                    self.publish_outputs(dt_sim=dt)

            next_step_wall += dt


    def cb_contact_diff_100hz(self):
        """
        100 Hz 타이머에서:
        - dt는 타이머 실제 호출 간격(실측) 사용
        - 최신 raw force(self._F_raw_latest)로 수치미분
        - 1차 LPF로 F, Fdot 필터링
        - filt force를 /contact_force_filt로 publish
        """

        now_ns = self.get_clock().now().nanoseconds

        # ---- 0) 첫 호출: 시간/prev force 초기화 ----
        if self._contact_timer_prev_ns is None:
            self._contact_timer_prev_ns = now_ns
            self._F_raw_prev = self._F_raw_latest.copy()
            # 필요하면 필터도 여기서 초기화할 수 있음(선택)
            # self._F_filt_latest = self._F_raw_prev.copy()
            # self._Fdot_filt_latest = np.zeros(3, dtype=float)
            return

        # ---- 1) dt 계산 (타이머 실측) ----
        dt = (now_ns - self._contact_timer_prev_ns) * 1e-9
        self._contact_timer_prev_ns = now_ns

        # 타이머 지터/중단 등 방어
        if dt <= 1e-6 or dt > 0.05:
            return

        # ---- 2) 최신 raw force 읽기 ----
        F = self._F_raw_latest  # numpy array (shared)
        # 안전하게 하려면 copy:
        # F = self._F_raw_latest.copy()

        # ---- 3) 수치미분 (Fdot) ----
        Fdot_raw = (F - self._F_raw_prev) / dt
        self._F_raw_prev = F.copy()

        self._Fdot_raw_latest = Fdot_raw.copy()


        # ---- 4) 1차 LPF alpha 계산
        wc = float(max(0.0, self.contact_cutoff_hz))

        if wc <= 0.0:
            a = 1.0
        else:
            a = float(np.clip(wc * dt, 0.0, 1.0))

        # ---- 5) 필터링 (Fdot, F) ----
        self._Fdot_filt_latest = (1.0 - a) * self._Fdot_filt_latest + a * Fdot_raw
        self._F_filt_latest    = (1.0 - a) * self._F_filt_latest    + a * F

        # ---- 6) publish (filtered force) ----
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.wrench.force.x = float(self._F_filt_latest[0])
        msg.wrench.force.y = float(self._F_filt_latest[1])
        msg.wrench.force.z = float(self._F_filt_latest[2])
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.pub_contact_force_filt.publish(msg)





    def viewer_loop(self):
        if self.viewer_hz <= 0:
            return

        viewer_dt = 1.0 / max(1e-9, self.viewer_hz)
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:


                self.get_logger().info("MuJoCo viewer started (passive).")
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    t0 = time.perf_counter()
                    with self._lock:
                        self._update_contact_resultant_arrow_in_viewer(viewer)
                        viewer.sync()
                    t1 = time.perf_counter()
                    sleep_t = viewer_dt - (t1 - t0)
                    if sleep_t > 0:
                        time.sleep(sleep_t)
        except Exception as e:
            self.get_logger().warn(f"viewer end: {e}")


    def close(self):
        self._stop = True

    
    @staticmethod
    def _contact_frame_to_world(con) -> np.ndarray:
        fr = np.array(con.frame, dtype=float).ravel()
        if fr.size == 9:
            # MuJoCo contact.frame은 [n; t1; t2]가 연속으로 저장되는 형태가 일반적이라
            # reshape(3,3) 하면 row가 axis가 된다.
            return fr.reshape(3, 3)
        return np.eye(3, dtype=float)




    @staticmethod
    def _arrow_mat_from_dir(d: np.ndarray) -> np.ndarray:
        z = d / (np.linalg.norm(d) + 1e-12)

        up = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(np.dot(z, up)) > 0.95:
            up = np.array([0.0, 1.0, 0.0], dtype=float)

        x = np.cross(up, z)
        x = x / (np.linalg.norm(x) + 1e-12)
        y = np.cross(z, x)

        # columns are axes
        return np.column_stack([x, y, z])



    @staticmethod
    def _set_geom_mat(g, R: np.ndarray):
        try:
            if getattr(g.mat, "shape", None) == (3, 3):
                g.mat[:, :] = R
            else:
                g.mat[:] = R.reshape(-1)
        except Exception:
            # 최후 fallback (그래도 안 되면 그냥 넘김)
            pass

    
    def mjfc(self, model, data):
        self.fcn = 0.0
        self.rf  = np.zeros(3, dtype=float)
        self.Fw  = np.zeros(3, dtype=float)

        for i in range(data.ncon):
            fci = np.zeros(6, dtype=float)
            try:
                mujoco.mj_contactForce(model, data, i, fci)

                con = data.contact[i]
                pos_w = np.array(con.pos, dtype=float)

                # con.frame -> (3,3)
                R = self._contact_frame_to_world(con)

                # ✅ MuJoCo frame 배열은 보통 [n; t1; t2] (rows = axes)
                n_w  = R[0, :]   # normal (world)
                t1_w = R[1, :]   # tangent1 (world)
                t2_w = R[2, :]   # tangent2 (world)

                # contact-frame force components (x=normal, y=t1, z=t2)
                F_c = fci[0:3].copy()

                # ✅ contact -> world : F_w = [n t1 t2] * F_c
                C = np.column_stack([n_w, t1_w, t2_w])   # columns are axes in world
                F_w = C @ F_c

                # (원하면 resultant 크기 weighting은 |normal|이 아니라 |F|로)
                Fn_mag = float(np.linalg.norm(F_w))
                if Fn_mag < 1e-12:
                    continue

                self.fcn += Fn_mag
                self.rf  += pos_w * Fn_mag
                self.Fw  += F_w

            except Exception:
                pass




    def _update_contact_resultant_arrow_in_viewer(self, viewer):
        if (not self.viz_contact_enable) or self.viz_contact_scale <= 0.0:
            viewer.user_scn.ngeom = 0
            return

        viewer.user_scn.ngeom = 0

        # aggregate contacts
        self.mjfc(self.model, self.data)

        if self.fcn <= 1e-12:
            return

        p0 = self.rf / self.fcn          # representative contact point (weighted)
        F  = self.Fw                     # resultant force (world)
        Fn = float(np.linalg.norm(F))
        if Fn <= 1e-12:
            return

        d = F / Fn
        length = self.viz_contact_scale * Fn

        g = viewer.user_scn.geoms[0]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3),
            np.zeros(3),
            np.zeros(9),
            np.array([1, 0, 0, 1], dtype=float)
        )

        g.pos[:] = p0
        R = self._arrow_mat_from_dir(d)
        self._set_geom_mat(g, R)
        g.size[:] = np.array([self.viz_contact_width, self.viz_contact_width, length], dtype=float)

        viewer.user_scn.ngeom = 1



def main():
    rclpy.init()
    node = CrazyfliePlant()
    signal.signal(signal.SIGINT, lambda *_: node.close())
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
