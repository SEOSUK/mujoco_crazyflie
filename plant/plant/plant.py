#!/usr/bin/env python3
import os
import time
import signal
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import mujoco
import mujoco.viewer

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray

# -------------------- rates --------------------
PHYSICS_HZ = 500.0          # MuJoCo integration rate
PUB_HZ = 200.0              # ROS publish rate (decimated from physics)
VIEWER_HZ = 60.0            # viewer sync rate (keep low to avoid blocking sim)


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


# -------------------- Node --------------------
class CrazyfliePlant(Node):
    """
    Sub:
      /crazyflie/in/input : Float32MultiArray [tau_x, tau_y, tau_z, Fz] (body frame)

    Pub:
      /crazyflie/out/pose        : PoseStamped (world, quat)                  <-- UNFILTERED attitude (GT quat normalized)
      /crazyflie/out/vel         : Vector3Stamped (world linear velocity)
      /crazyflie/out/ang_vel     : Vector3Stamped (body angular velocity)     <-- RAW gyro (UNFILTERED)
      /crazyflie/out/acc         : Vector3Stamped (world linear acceleration)
      /crazyflie/out/ang_acc     : Vector3Stamped (body angular acceleration) <-- numerical diff of RAW gyro
      /crazyflie/out/ang_vel_gt  : Vector3Stamped (body angular velocity)     <-- from qvel (UNFILTERED)
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

        # ---- numerical diff state (RAW gyro) ----
        self._prev_gyro_raw_B: Optional[np.ndarray] = None

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
        self.pub_angvel = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel", 10)     # RAW gyro
        self.pub_acc = self.create_publisher(Vector3Stamped, "/crazyflie/out/acc", 10)
        self.pub_angacc = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_acc", 10)    # diff of RAW gyro
        self.pub_angvel_gt = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel_gt", 10)

        # ---- threads start ----
        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.viewer_thread.start()
        self.sim_thread.start()

        self.get_logger().info(
            f"Running: physics={self.physics_hz:.1f}Hz, pub={self.pub_hz:.1f}Hz, viewer={self.viewer_hz:.1f}Hz"
        )
        self.get_logger().info("Fix applied: publish is STEP-BASED (no publish catch-up without mj_step).")
        self.get_logger().info("LPF removed: publishing UNFILTERED attitude and RAW gyro.")

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

        # use measured quat (normalized) for accel transform, and publish pose as normalized quat
        quat_wxyz = quat_normalize_wxyz(quat_wxyz_meas)

        linacc_W = self.read_imu_acc_world(quat_wxyz)
        gyro_B = self.read_imu_gyro_body()  # RAW

        dt = max(1e-6, float(dt_sim))

        # numerical derivative of RAW gyro (expect noisy)
        if self._prev_gyro_raw_B is None:
            angacc_B = np.zeros(3, dtype=float)
        else:
            angacc_B = (gyro_B - self._prev_gyro_raw_B) / dt
        self._prev_gyro_raw_B = gyro_B.copy()

        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(pos_W[0])
        pose_msg.pose.position.y = float(pos_W[1])
        pose_msg.pose.position.z = float(pos_W[2])
        q_xyzw = quat_wxyz_to_xyzw(quat_wxyz)
        pose_msg.pose.orientation.x = float(q_xyzw[0])
        pose_msg.pose.orientation.y = float(q_xyzw[1])
        pose_msg.pose.orientation.z = float(q_xyzw[2])
        pose_msg.pose.orientation.w = float(q_xyzw[3])
        self.pub_pose.publish(pose_msg)

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "world"
        vel_msg.vector.x = float(linvel_W[0])
        vel_msg.vector.y = float(linvel_W[1])
        vel_msg.vector.z = float(linvel_W[2])
        self.pub_vel.publish(vel_msg)

        w_msg = Vector3Stamped()
        w_msg.header.stamp = stamp
        w_msg.header.frame_id = "body"
        w_msg.vector.x = float(gyro_B[0])
        w_msg.vector.y = float(gyro_B[1])
        w_msg.vector.z = float(gyro_B[2])
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
        a_msg.vector.x = float(angacc_B[0])
        a_msg.vector.y = float(angacc_B[1])
        a_msg.vector.z = float(angacc_B[2])
        self.pub_angacc.publish(a_msg)

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

                # publish only on step boundaries (no publish catch-up)
                if (step_count % pub_decim) == 0:
                    self.publish_outputs(dt_sim=dt)

            next_step_wall += dt

    def viewer_loop(self):
        # keep viewer from stealing the sim lock too often
        if self.viewer_hz <= 0:
            return

        viewer_dt = 1.0 / max(1e-9, self.viewer_hz)
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                self.get_logger().info("MuJoCo viewer started (passive).")
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    t0 = time.perf_counter()
                    with self._lock:
                        viewer.sync()
                    t1 = time.perf_counter()
                    # sleep to target viewer_hz
                    sleep_t = viewer_dt - (t1 - t0)
                    if sleep_t > 0:
                        time.sleep(sleep_t)
        except Exception as e:
            self.get_logger().warn(f"viewer end: {e}")

    def close(self):
        self._stop = True


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
