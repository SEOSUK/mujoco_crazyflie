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
        self.declare_parameter("noise.pos_var", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.vel_var", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.att_var", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.ang_vel_var", [0.0, 0.0, 0.0])
        self.declare_parameter("noise.ang_acc_var", [0.0, 0.0, 0.0])

        self.noise_enable = bool(self.get_parameter("noise.enable").value)
        seed = int(self.get_parameter("noise.seed").value)
        self.rng = np.random.default_rng(None if seed == 0 else seed)

        self._pos_std = self._var_to_std3(self.get_parameter("noise.pos_var").value)
        self._vel_std = self._var_to_std3(self.get_parameter("noise.vel_var").value)
        self._att_std = self._var_to_std3(self.get_parameter("noise.att_var").value)
        self._ang_vel_std = self._var_to_std3(self.get_parameter("noise.ang_vel_var").value)
        self._ang_acc_std = self._var_to_std3(self.get_parameter("noise.ang_acc_var").value)

        pkg_share = get_package_share_directory("plant")
        xml_path = os.path.join(pkg_share, "data", "cf21B_500.xml")

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

        self._lock = threading.Lock()
        self._stop = False
        self._prev_gyro_used_B: Optional[np.ndarray] = None

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

        self.sub_input = self.create_subscription(
            Float32MultiArray, "/crazyflie/in/input", self.cb_input, 10
        )

        self.pub_pose = self.create_publisher(PoseStamped, "/crazyflie/out/pose", 10)
        self.pub_vel = self.create_publisher(Vector3Stamped, "/crazyflie/out/vel", 10)
        self.pub_angvel = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel", 10)
        self.pub_acc = self.create_publisher(Vector3Stamped, "/crazyflie/out/acc", 10)
        self.pub_angacc = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_acc", 10)
        self.pub_angvel_gt = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel_gt", 10)

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
    def _var_to_std3(v: Union[float, Sequence[float]]) -> np.ndarray:
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
        f = self.B_pinv @ w
        f_clip = np.clip(f, self.thrust_min, self.thrust_max)

        if all(v is not None and v >= 0 for v in self.act_force_ids):
            for i in range(4):
                self.data.ctrl[self.act_force_ids[i]] = float(f_clip[i])
        else:
            self.data.ctrl[0:4] = f_clip

        tau_m = self.motor_dir * self.k_tau * f_clip
        if all(v is not None and v >= 0 for v in self.act_torque_ids):
            for i in range(4):
                self.data.ctrl[self.act_torque_ids[i]] = float(tau_m[i])

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

        pos_W_noisy = pos_W + self._randn3(self._pos_std)

        dtheta_B = self._randn3(self._att_std)
        dq = rotvec_to_quat_wxyz(dtheta_B)
        quat_wxyz_noisy = quat_normalize_wxyz(quat_mul_wxyz(quat_wxyz, dq))

        linacc_W = self.read_imu_acc_world(quat_wxyz_noisy)

        gyro_B = self.read_imu_gyro_body()
        gyro_B_used = gyro_B + self._randn3(self._ang_vel_std)

        linvel_W_noisy = linvel_W + self._randn3(self._vel_std)

        dt = max(1e-6, float(dt_sim))
        if self._prev_gyro_used_B is None:
            angacc_B = np.zeros(3, dtype=float)
        else:
            angacc_B = (gyro_B_used - self._prev_gyro_used_B) / dt
        self._prev_gyro_used_B = gyro_B_used.copy()

        angacc_B_noisy = angacc_B + self._randn3(self._ang_acc_std)

        stamp = self.get_clock().now().to_msg()

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

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "world"
        vel_msg.vector.x = float(linvel_W_noisy[0])
        vel_msg.vector.y = float(linvel_W_noisy[1])
        vel_msg.vector.z = float(linvel_W_noisy[2])
        self.pub_vel.publish(vel_msg)

        w_msg = Vector3Stamped()
        w_msg.header.stamp = stamp
        w_msg.header.frame_id = "body"
        w_msg.vector.x = float(gyro_B_used[0])
        w_msg.vector.y = float(gyro_B_used[1])
        w_msg.vector.z = float(gyro_B_used[2])
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
        a_msg.vector.x = float(angacc_B_noisy[0])
        a_msg.vector.y = float(angacc_B_noisy[1])
        a_msg.vector.z = float(angacc_B_noisy[2])
        self.pub_angacc.publish(a_msg)

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