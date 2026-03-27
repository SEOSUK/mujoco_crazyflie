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

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray

from .contact import ContactManager
from collections import deque

PHYSICS_HZ = 1000.0
PUB_HZ = 400.0
VIEWER_HZ = 60.0


def quat_wxyz_to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = q_wxyz
    return np.array([x, y, z, w], dtype=float)


def rotmat_from_quat_wxyz(q: np.ndarray) -> np.ndarray:
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
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=float)


def rotvec_to_quat_wxyz(r: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(r))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    axis = r / angle
    half = 0.5 * angle
    s = np.sin(half)
    return np.array([np.cos(half), axis[0]*s, axis[1]*s, axis[2]*s], dtype=float)


class CrazyfliePlant(Node):
    def __init__(self):
        super().__init__("mujoco_crazyflie_plant")

        self.declare_parameter("physics_hz", PHYSICS_HZ)
        self.declare_parameter("pub_hz", PUB_HZ)
        self.declare_parameter("viewer_hz", VIEWER_HZ)

        self.physics_hz = float(self.get_parameter("physics_hz").value)
        self.pub_hz = float(self.get_parameter("pub_hz").value)
        self.viewer_hz = float(self.get_parameter("viewer_hz").value)

        self.declare_parameter("arm_xy", 0.035355)
        self.declare_parameter("k_tau", 0.00594)
        self.declare_parameter("motor_dir", [1.0, -1.0, 1.0, -1.0])
        self.declare_parameter("thrust_min", 0.0)
        self.declare_parameter("thrust_max", 0.20)

        self.a = float(self.get_parameter("arm_xy").value)
        self.k_tau = float(self.get_parameter("k_tau").value)
        self.motor_dir = np.array(self.get_parameter("motor_dir").value, dtype=float)
        self.thrust_min = float(self.get_parameter("thrust_min").value)
        self.thrust_max = float(self.get_parameter("thrust_max").value)
        if self.motor_dir.shape[0] != 4:
            self.motor_dir = np.array([1.0, -1.0, 1.0, -1.0], dtype=float)

        self.declare_parameter("noise.enable", True)
        self.declare_parameter("noise.seed", 0)
        self.declare_parameter("noise.hz", 60.0)
        self.declare_parameter("noise.pos_amp", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.vel_amp", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.att_amp", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.ang_vel_amp", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.ang_acc_amp", [0.0, 0.0, 0.0])


        self.noise_enable = bool(self.get_parameter("noise.enable").value)
        seed = int(self.get_parameter("noise.seed").value)
        self.rng = np.random.default_rng(None if seed == 0 else seed)

        # 변경: var를 "진폭"으로 사용
        self._pos_amp = self._to_amp3(self.get_parameter("noise.pos_amp").value)
        self._vel_amp = self._to_amp3(self.get_parameter("noise.vel_amp").value)
        self._att_amp = self._to_amp3(self.get_parameter("noise.att_amp").value)
        self._ang_vel_amp = self._to_amp3(self.get_parameter("noise.ang_vel_amp").value)
        self._ang_acc_amp = self._to_amp3(self.get_parameter("noise.ang_acc_amp").value)
        
        self.noise_hz = float(self.get_parameter("noise.hz").value)

        self._noise_t0_ns = None

        self._pos_noise = np.zeros(3, dtype=float)
        self._vel_noise = np.zeros(3, dtype=float)
        self._att_noise = np.zeros(3, dtype=float)
        self._ang_vel_noise = np.zeros(3, dtype=float)
        self._ang_acc_noise = np.zeros(3, dtype=float)

        # 축별 위상: 완전히 같은 사인파가 되지 않도록 seed 기반 랜덤 위상 부여
        self._pos_phase = self.rng.uniform(0.0, 2.0*np.pi, size=3)
        self._vel_phase = self.rng.uniform(0.0, 2.0*np.pi, size=3)
        self._att_phase = self.rng.uniform(0.0, 2.0*np.pi, size=3)
        self._ang_vel_phase = self.rng.uniform(0.0, 2.0*np.pi, size=3)
        self._ang_acc_phase = self.rng.uniform(0.0, 2.0*np.pi, size=3)


        self.declare_parameter("noise.lpf.enable", False)
        self.declare_parameter("noise.lpf.cutoff_hz", 0.0)

        self.noise_lpf_enable = bool(self.get_parameter("noise.lpf.enable").value)
        self.noise_lpf_cutoff_hz = float(self.get_parameter("noise.lpf.cutoff_hz").value)

        self._pos_filt = np.zeros(3, dtype=float)
        self._vel_filt = np.zeros(3, dtype=float)
        self._ang_vel_filt = np.zeros(3, dtype=float)
        self._ang_acc_filt = np.zeros(3, dtype=float)

        self._noise_lpf_initialized = False


        pkg_share = get_package_share_directory("plant")
        xml_path = os.path.join(pkg_share, "data", "scene.xml")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.bid_drone = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "drone")
        if self.bid_drone < 0:
            raise RuntimeError("Body 'drone' not found in XML")

        jnt_adr = int(self.model.body_jntadr[self.bid_drone])
        jnt_num = int(self.model.body_jntnum[self.bid_drone])
        if jnt_num < 1:
            raise RuntimeError("Body 'drone' has no joint (expected freejoint)")

        self.jid_drone = jnt_adr
        self.qpos_adr_drone = int(self.model.jnt_qposadr[self.jid_drone])
        self.qvel_adr_drone = int(self.model.jnt_dofadr[self.jid_drone])
        jtype = int(self.model.jnt_type[self.jid_drone])
        if jtype != mujoco.mjtJoint.mjJNT_FREE:
            raise RuntimeError(f"Body 'drone' first joint is not FREE (type={jtype})")

        self.model.opt.timestep = 1.0 / max(1e-9, self.physics_hz)

        self.imu_acc_sid = self._sensor_id("imu_acc")
        self.imu_gyro_sid = self._sensor_id("imu_gyro")

    # 여기가 allocation 만드는 부분
        a = self.a
        x = np.array([+a, -a, -a, +a], dtype=float)
        y = np.array([-a, -a, +a, +a], dtype=float)
        d = self.motor_dir
        k = self.k_tau

        self.B = np.vstack([
            y,
            -x,
            d * k,
            np.ones(4),
        ]).astype(float)
        self.B_pinv = np.linalg.pinv(self.B)

        self.u = np.zeros(4, dtype=float)
        # actuator delay / lag
        self.declare_parameter("actuator.delay.enable", True)
        self.declare_parameter("actuator.delay.sec", 0.0)
        self.declare_parameter("actuator.lpf.enable", True)
        self.declare_parameter("actuator.lpf.cutoff_hz", 0.0)

        self.act_delay_enable = bool(self.get_parameter("actuator.delay.enable").value)
        self.act_delay_sec = float(self.get_parameter("actuator.delay.sec").value)
        self.act_lpf_enable = bool(self.get_parameter("actuator.lpf.enable").value)
        self.act_lpf_cutoff_hz = float(self.get_parameter("actuator.lpf.cutoff_hz").value)

        self.dt = 1.0 / max(1e-9, self.physics_hz)

        self._f_act = np.zeros(4, dtype=float)
        self.last_motor_thrust = np.zeros(4, dtype=float)
        
        self._delay_steps = 0
        if self.act_delay_enable and self.act_delay_sec > 0.0:
            self._delay_steps = max(0, int(round(self.act_delay_sec / self.dt)))

        self._f_delay_buf = deque(
            [np.zeros(4, dtype=float) for _ in range(self._delay_steps + 1)],
            maxlen=self._delay_steps + 1
        )

        self._lock = threading.Lock()
        self._stop = False
        self._prev_gyro_used_B: Optional[np.ndarray] = None

        def _act_id(name: str) -> Optional[int]:
            try:
                return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            except Exception:
                return None

        self.act_ids = []
        for i in range(4):
            self.act_ids.append(_act_id(f"motor{i}"))

        self.sub_input = self.create_subscription(
            Float32MultiArray, "/crazyflie/in/input", self.cb_input, 10
        )

        self.pub_pose = self.create_publisher(PoseStamped, "/crazyflie/out/pose", 10)
        self.pub_vel = self.create_publisher(Vector3Stamped, "/crazyflie/out/vel", 10)
        self.pub_angvel = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel", 10)
        self.pub_acc = self.create_publisher(Vector3Stamped, "/crazyflie/out/acc", 10)
        self.pub_angacc = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_acc", 10)
        self.pub_angvel_gt = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel_gt", 10)
        self.pub_motor_thrust = self.create_publisher(
            Float32MultiArray, "/crazyflie/out/motor_thrust", 10
        )
        self.pub_contact_force = self.create_publisher(
            __import__("geometry_msgs.msg", fromlist=["WrenchStamped"]).WrenchStamped,
            "/crazyflie/out/EE_contact_force",
            10,
        )
        self.pub_contact_force_filt = self.create_publisher(
            __import__("geometry_msgs.msg", fromlist=["WrenchStamped"]).WrenchStamped,
            "/crazyflie/out/EE_contact_force_filt",
            10,
        )

        self.contact = ContactManager(
            node=self,
            model=self.model,
            data=self.data,
            pub_contact_force=self.pub_contact_force,
            pub_contact_force_filt=self.pub_contact_force_filt,
        )

        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.viewer_thread.start()
        self.sim_thread.start()

    @staticmethod
    def _to_amp3(v: Union[float, Sequence[float]]) -> np.ndarray:
        if isinstance(v, (int, float)):
            amp = np.array([float(v)] * 3, dtype=float)
        else:
            arr = np.array(list(v), dtype=float)
            if arr.size == 1:
                amp = np.array([float(arr[0])] * 3, dtype=float)
            else:
                amp = arr.reshape(3)
        amp = np.clip(amp, 0.0, None)
        return amp


    def _update_noise_sine(self):
        if not self.noise_enable:
            self._pos_noise[:] = 0.0
            self._vel_noise[:] = 0.0
            self._att_noise[:] = 0.0
            self._ang_vel_noise[:] = 0.0
            self._ang_acc_noise[:] = 0.0
            return

        now_ns = self.get_clock().now().nanoseconds

        if self._noise_t0_ns is None:
            self._noise_t0_ns = now_ns

        t = (now_ns - self._noise_t0_ns) * 1e-9
        w = 2.0 * np.pi * self.noise_hz

        self._pos_noise = self._pos_amp * np.sin(w * t + self._pos_phase)
        self._vel_noise = self._vel_amp * np.sin(w * t + self._vel_phase)
        self._att_noise = self._att_amp * np.sin(w * t + self._att_phase)
        self._ang_vel_noise = self._ang_vel_amp * np.sin(w * t + self._ang_vel_phase)
        self._ang_acc_noise = self._ang_acc_amp * np.sin(w * t + self._ang_acc_phase)

    def _apply_noise_lpf(self, x: np.ndarray, x_filt: np.ndarray) -> np.ndarray:
        if (not self.noise_lpf_enable) or self.noise_lpf_cutoff_hz <= 1e-9:
            return x.copy()

        wc = 2.0 * np.pi * self.noise_lpf_cutoff_hz
        alpha = 1.0 - np.exp(-wc * self.dt)
        return (1.0 - alpha) * x_filt + alpha * x

    def _sensor_id(self, name: str) -> Optional[int]:
        try:
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            return sid
        except Exception:
            return None

    def _read_sensor_vec3(self, sid: Optional[int]) -> np.ndarray:
        if sid is None:
            return np.zeros(3, dtype=float)
        adr = int(self.model.sensor_adr[sid])
        dim = int(self.model.sensor_dim[sid])
        return np.array(self.data.sensordata[adr:adr + dim], dtype=float)

    def read_imu_acc_world(self, quat_wxyz: np.ndarray) -> np.ndarray:
        acc_B = self._read_sensor_vec3(self.imu_acc_sid)
        R_BW = rotmat_from_quat_wxyz(quat_wxyz)
        g_W = np.array(self.model.opt.gravity, dtype=float)
        return R_BW @ acc_B + g_W

    def read_imu_gyro_body(self) -> np.ndarray:
        return self._read_sensor_vec3(self.imu_gyro_sid)

    def cb_input(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        with self._lock:
            self.u[:] = np.array(msg.data[:4], dtype=float)

    def apply_control(self):
        tau_x, tau_y, tau_z, Fz = self.u

        w = np.array([tau_x, tau_y, tau_z, Fz], dtype=float)
        f_cmd = self.B_pinv @ w
        f_cmd = np.clip(f_cmd, self.thrust_min, self.thrust_max)

        # actuator dynamics
        f_applied = self.apply_actuator_dynamics(f_cmd)
        f_applied = np.clip(f_applied, self.thrust_min, self.thrust_max)

        self.last_motor_thrust = f_applied.copy()

        if all(v is not None and v >= 0 for v in self.act_ids):
            for i in range(4):
                self.data.ctrl[self.act_ids[i]] = float(f_applied[i])
        else:
            self.data.ctrl[0:4] = f_applied
            
    def apply_actuator_dynamics(self, f_cmd: np.ndarray) -> np.ndarray:
        f_in = np.array(f_cmd, dtype=float).copy()

        # 1) pure delay
        if self.act_delay_enable and self._delay_steps > 0:
            self._f_delay_buf.append(f_in)
            f_delayed = self._f_delay_buf[0].copy()
        else:
            f_delayed = f_in

        # 2) 1st-order lag
        if self.act_lpf_enable and self.act_lpf_cutoff_hz > 1e-9:
            wc = 2.0 * np.pi * self.act_lpf_cutoff_hz
            alpha = 1.0 - np.exp(-wc * self.dt)
            self._f_act = (1.0 - alpha) * self._f_act + alpha * f_delayed
        else:
            self._f_act = f_delayed.copy()

        return self._f_act.copy()
    

    def read_state(self):
        qa = self.qpos_adr_drone
        va = self.qvel_adr_drone

        pos_W = np.array(self.data.qpos[qa:qa+3], dtype=float)
        quat_wxyz = np.array(self.data.qpos[qa+3:qa+7], dtype=float)

        linvel_W = np.array(self.data.qvel[va:va+3], dtype=float)
        angvel_W = np.array(self.data.qvel[va+3:va+6], dtype=float)

        R_BW = rotmat_from_quat_wxyz(quat_wxyz)
        angvel_B = R_BW.T @ angvel_W
        return pos_W, quat_wxyz, linvel_W, angvel_B

    def publish_outputs(self, dt_sim: float):
        pos_W, quat_wxyz_meas, linvel_W, angvel_B_gt = self.read_state()
        quat_wxyz = quat_normalize_wxyz(quat_wxyz_meas)

        self._update_noise_sine()

        pos_W_noisy = pos_W + self._pos_noise

        dtheta_B = self._att_noise
        dq = rotvec_to_quat_wxyz(dtheta_B)
        quat_wxyz_noisy = quat_normalize_wxyz(quat_mul_wxyz(quat_wxyz, dq))

        linacc_W = self.read_imu_acc_world(quat_wxyz_noisy)

        gyro_B = self.read_imu_gyro_body()
        gyro_B_used = gyro_B + self._ang_vel_noise

        linvel_W_noisy = linvel_W + self._vel_noise


        if not self._noise_lpf_initialized:
            self._pos_filt = pos_W_noisy.copy()
            self._vel_filt = linvel_W_noisy.copy()
            self._ang_vel_filt = gyro_B_used.copy()
            self._noise_lpf_initialized = True
        else:
            self._pos_filt = self._apply_noise_lpf(pos_W_noisy, self._pos_filt)
            self._vel_filt = self._apply_noise_lpf(linvel_W_noisy, self._vel_filt)
            self._ang_vel_filt = self._apply_noise_lpf(gyro_B_used, self._ang_vel_filt)


        dt = max(1e-6, float(dt_sim))
        if self._prev_gyro_used_B is None:
            angacc_B = np.zeros(3, dtype=float)
        else:
            angacc_B = (gyro_B_used - self._prev_gyro_used_B) / dt
        self._prev_gyro_used_B = gyro_B_used.copy()

        angacc_B_noisy = angacc_B + self._ang_acc_noise


        if not self._noise_lpf_initialized:
            self._ang_acc_filt = angacc_B_noisy.copy()
        else:
            self._ang_acc_filt = self._apply_noise_lpf(angacc_B_noisy, self._ang_acc_filt)


        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(self._pos_filt[0])
        pose_msg.pose.position.y = float(self._pos_filt[1])
        pose_msg.pose.position.z = float(self._pos_filt[2])
        q_xyzw = quat_wxyz_to_xyzw(quat_wxyz_noisy)
        pose_msg.pose.orientation.x = float(q_xyzw[0])
        pose_msg.pose.orientation.y = float(q_xyzw[1])
        pose_msg.pose.orientation.z = float(q_xyzw[2])
        pose_msg.pose.orientation.w = float(q_xyzw[3])
        self.pub_pose.publish(pose_msg)

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "world"
        vel_msg.vector.x = float(self._vel_filt[0])
        vel_msg.vector.y = float(self._vel_filt[1])
        vel_msg.vector.z = float(self._vel_filt[2])
        self.pub_vel.publish(vel_msg)

        w_msg = Vector3Stamped()
        w_msg.header.stamp = stamp
        w_msg.header.frame_id = "body"
        w_msg.vector.x = float(self._ang_vel_filt[0])
        w_msg.vector.y = float(self._ang_vel_filt[1])
        w_msg.vector.z = float(self._ang_vel_filt[2])
        self.pub_angvel.publish(w_msg)

        wgt_msg = Vector3Stamped()
        wgt_msg.header.stamp = stamp
        wgt_msg.header.frame_id = "body"
        wgt_msg.vector.x = float(angvel_B_gt[0])
        wgt_msg.vector.y = float(angvel_B_gt[1])
        wgt_msg.vector.z = float(angvel_B_gt[2])
        self.pub_angvel_gt.publish(wgt_msg)

        acc_msg = Vector3Stamped()
        acc_msg.header.stamp = stamp
        acc_msg.header.frame_id = "world"
        acc_msg.vector.x = float(linacc_W[0])
        acc_msg.vector.y = float(linacc_W[1])
        acc_msg.vector.z = float(linacc_W[2])
        self.pub_acc.publish(acc_msg)

        a_msg = Vector3Stamped()
        a_msg.header.stamp = stamp
        a_msg.header.frame_id = "body"
        a_msg.vector.x = float(self._ang_acc_filt[0])
        a_msg.vector.y = float(self._ang_acc_filt[1])
        a_msg.vector.z = float(self._ang_acc_filt[2])
        self.pub_angacc.publish(a_msg)

        motor_msg = Float32MultiArray()
        motor_msg.data = self.last_motor_thrust.astype(np.float32).tolist()
        self.pub_motor_thrust.publish(motor_msg)

        self.contact.update_raw_and_publish(stamp)

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

    def viewer_loop(self):
        if self.viewer_hz <= 0:
            return

        viewer_dt = 1.0 / max(1e-9, self.viewer_hz)
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    t0 = time.perf_counter()
                    with self._lock:
                        self.contact.update_contact_resultant_arrow_in_viewer(viewer)
                        viewer.sync()
                    t1 = time.perf_counter()
                    sleep_t = viewer_dt - (t1 - t0)
                    if sleep_t > 0:
                        time.sleep(sleep_t)
        except Exception:
            pass

    def close(self):
        self._stop = True
        self.contact.close()


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