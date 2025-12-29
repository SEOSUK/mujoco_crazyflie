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

PHYSICS_HZ = 400.0


def quat_wxyz_to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
    # MuJoCo free joint quaternion: (w, x, y, z)
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


class CrazyfliePlant(Node):
    """
    Sub:
      /crazyflie/in/input : Float32MultiArray [tau_x, tau_y, tau_z, Fz] (body frame)

    Pub:
      /crazyflie/out/pose     : PoseStamped (world, quat)
      /crazyflie/out/vel      : Vector3Stamped (world linear velocity)
      /crazyflie/out/ang_vel  : Vector3Stamped (body angular velocity)
      /crazyflie/out/acc      : Vector3Stamped (world linear acceleration)
      /crazyflie/out/ang_acc  : Vector3Stamped (body angular acceleration)
    """

    def __init__(self):
        super().__init__("mujoco_crazyflie_plant")

        pkg_share = get_package_share_directory("plant")
        xml_path = os.path.join(pkg_share, "data", "cf21B_500.xml")
        self.get_logger().info(f"Loading MuJoCo model: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYSICS_HZ

        # -------- IMU sensor ids --------
        try:
            self.imu_acc_sid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_acc"
            )
            self.get_logger().info("IMU accelerometer sensor found.")
        except Exception:
            self.imu_acc_sid = None
            self.get_logger().warn("IMU accelerometer sensor NOT found.")

        # Parameters for mixer (X config)
        # Motor sites from your xml:
        # motor0: (+a, -a), motor1: (-a, -a), motor2: (-a, +a), motor3: (+a, +a)
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

        # Build physical allocation matrix B:
        # [tau_x]   [ y0  y1  y2  y3 ] [f0]
        # [tau_y] = [-x0 -x1 -x2 -x3 ] [f1]
        # [tau_z]   [ d0k d1k d2k d3k] [f2]
        # [ Fz  ]   [  1   1   1   1 ] [f3]
        #
        # with x,y = (+a,-a), (-a,-a), (-a,+a), (+a,+a)
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

        # Precompute pseudo-inverse (4x4 invertible if k_tau != 0)
        # Use pinv for numerical safety
        self.B_pinv = np.linalg.pinv(self.B)


        # IO
        self.u = np.zeros(4, dtype=float)  # [tau_x, tau_y, tau_z, Fz]

        self._lock = threading.Lock()
        self._stop = False

        self._prev_t: Optional[float] = None
        self._prev_linvel_W = None
        self._prev_angvel_B = None

        # -------- actuator index lookup (by name) --------
        def _act_id(name: str) -> Optional[int]:
            try:
                return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            except Exception:
                return None

        self.act_force_ids = []
        self.act_torque_ids = []

        for i in range(4):
            fid = _act_id(f"motor{i}_force")
            tid = _act_id(f"motor{i}_torque")
            self.act_force_ids.append(fid)
            self.act_torque_ids.append(tid)

        # Check availability
        if any(v is None or v < 0 for v in self.act_force_ids):
            self.get_logger().warn(
                f"Could not find all motor*_force actuators by name. act_force_ids={self.act_force_ids}. "
                "Will fall back to ctrl[0:4]."
            )

        if any(v is None or v < 0 for v in self.act_torque_ids):
            self.get_logger().warn(
                f"Could not find all motor*_torque actuators by name. act_torque_ids={self.act_torque_ids}. "
                "Reaction torque will NOT be applied unless torque actuators exist in XML."
            )
        else:
            self.get_logger().info(f"Found torque actuators: {self.act_torque_ids}")



        # ROS
        self.sub_input = self.create_subscription(
            Float32MultiArray, "/crazyflie/in/input", self.cb_input, 10
        )

        self.pub_pose = self.create_publisher(PoseStamped, "/crazyflie/out/pose", 10)
        self.pub_vel = self.create_publisher(Vector3Stamped, "/crazyflie/out/vel", 10)
        self.pub_angvel = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_vel", 10)
        self.pub_acc = self.create_publisher(Vector3Stamped, "/crazyflie/out/acc", 10)
        self.pub_angacc = self.create_publisher(Vector3Stamped, "/crazyflie/out/ang_acc", 10)

        # threads
        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.viewer_thread.start()
        self.sim_thread.start()

        self.get_logger().info("Physical allocation enabled: tau_z is produced ONLY via k_tau*f_i.")

    def read_imu_acc_world(self, quat_wxyz):
        """
        Returns world-frame linear acceleration from MuJoCo IMU.
        """
        if self.imu_acc_sid is None:
            return np.zeros(3, dtype=float)

        adr = self.model.sensor_adr[self.imu_acc_sid]
        dim = self.model.sensor_dim[self.imu_acc_sid]

        acc_B = np.array(
            self.data.sensordata[adr:adr + dim], dtype=float
        )  # body frame, includes gravity

        # rotation: body -> world
        R_BW = rotmat_from_quat_wxyz(quat_wxyz)

        # world gravity (MuJoCo convention)
        g_W = np.array(self.model.opt.gravity, dtype=float)

        # world linear acceleration
        acc_W = R_BW @ acc_B + g_W
        return acc_W

    def cb_input(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn("'/crazyflie/in/input' needs 4 floats: [tau_x, tau_y, tau_z, Fz]")
            return
        with self._lock:
            self.u[:] = np.array(msg.data[:4], dtype=float)

    # ------------------------ Control Allocation (physical) ------------------------
    def apply_control(self):
        tau_x, tau_y, tau_z, Fz = self.u

        # Solve for motor thrusts
        w = np.array([tau_x, tau_y, tau_z, Fz], dtype=float)
        f = self.B_pinv @ w  # (4,)

        # clip thrusts to [min, max]
        f_clip = np.clip(f, self.thrust_min, self.thrust_max)

        if np.any(np.abs(f_clip - f) > 1e-9):
            self.get_logger().warn(f"Thrust clipped. raw={f.tolist()} clipped={f_clip.tolist()}")

        # ---------- apply thrust ----------
        # Prefer actuator-name mapping if available
        if all(v is not None and v >= 0 for v in self.act_force_ids):
            for i in range(4):
                self.data.ctrl[self.act_force_ids[i]] = float(f_clip[i])
        else:
            # fallback: assume first 4 actuators are motor forces
            self.data.ctrl[0:4] = f_clip

        # ---------- apply reaction torque (per motor) ----------
        # motor reaction torque per motor: tau_i = dir_i * k_tau * f_i
        tau_m = self.motor_dir * self.k_tau * f_clip  # (4,)

        if all(v is not None and v >= 0 for v in self.act_torque_ids):
            for i in range(4):
                self.data.ctrl[self.act_torque_ids[i]] = float(tau_m[i])
        else:
            # If torque actuators do not exist, we cannot display motor*_torque nor generate yaw torque
            # through actuator channels. (MuJoCo does not automatically add reaction torque.)
            pass


    # ------------------------ State publish ------------------------
    def read_state(self):
        pos_W = np.array(self.data.qpos[0:3], dtype=float)
        quat_wxyz = np.array(self.data.qpos[3:7], dtype=float)

        linvel_W = np.array(self.data.qvel[0:3], dtype=float)
        angvel_W = np.array(self.data.qvel[3:6], dtype=float)

        R_BW = rotmat_from_quat_wxyz(quat_wxyz)
        angvel_B = R_BW.T @ angvel_W

        return pos_W, quat_wxyz, linvel_W, angvel_B

    def publish_outputs(self, now_wall: float):
        pos_W, quat_wxyz, linvel_W, angvel_B = self.read_state()

        # -------- linear acceleration from IMU --------
        linacc_W = self.read_imu_acc_world(quat_wxyz)

        # -------- angular acceleration (keep numerical diff if you want) --------
        if self._prev_t is None:
            angacc_B = np.zeros(3, dtype=float)
        else:
            dt = max(1e-6, now_wall - self._prev_t)
            angacc_B = (angvel_B - self._prev_angvel_B) / dt

        self._prev_t = now_wall
        self._prev_linvel_W = linvel_W.copy()
        self._prev_angvel_B = angvel_B.copy()

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
        w_msg.vector.x = float(angvel_B[0])
        w_msg.vector.y = float(angvel_B[1])
        w_msg.vector.z = float(angvel_B[2])
        self.pub_angvel.publish(w_msg)

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
        next_step = time.perf_counter()
        next_pub = next_step

        while rclpy.ok() and not self._stop:
            now = time.perf_counter()

            with self._lock:
                self.apply_control()

                while now >= next_step:
                    mujoco.mj_step(self.model, self.data)
                    next_step += 1.0 / PHYSICS_HZ

                while now >= next_pub:
                    self.publish_outputs(now_wall=now)
                    next_pub += 1.0 / PHYSICS_HZ

            sleep_t = next_step - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)

    def viewer_loop(self):
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                self.get_logger().info("MuJoCo viewer started (passive).")
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    with self._lock:
                        viewer.sync()
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
