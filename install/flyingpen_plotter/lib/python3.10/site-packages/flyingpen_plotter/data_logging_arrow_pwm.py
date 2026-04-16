#!/usr/bin/env python3
from __future__ import annotations

import math
import signal
import sys
import threading
from collections import deque
from typing import Dict, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
    QFrame,
    QGridLayout,
)

import pyqtgraph as pg

from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped, QuaternionStamped
from std_msgs.msg import Float32, Float32MultiArray


def safe_unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.zeros_like(v, dtype=float)
    return v / n


def quat_xyzw_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)

    qx /= n
    qy /= n
    qz /= n
    qw /= n

    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz),       2.0 * (qx * qz + qw * qy)],
        [2.0 * (qx * qy + qw * qz),       1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)],
        [2.0 * (qx * qz - qw * qy),       2.0 * (qy * qz + qw * qx),       1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=float)


class ROSDataBuffer(Node):
    def __init__(self):
        super().__init__("data_logging_arrow_pwm")

        self.declare_parameter("history_sec", 5.0)
        self.declare_parameter("update_hz", 30.0)

        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(100, int(self.history_sec * self.update_hz) + 20)

        keys = [
            "force_cmd",
            "force_act",
            "pwm1", "pwm2", "pwm3", "pwm4",
            "drone_acc_x", "drone_acc_y", "drone_acc_z",
            "ee_acc_x", "ee_acc_y", "ee_acc_z",
        ]

        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for k in keys:
            self.data[k] = deque(maxlen=self.maxlen)

        self.latest_scalar = {k: np.nan for k in keys}

        self.ee_pos = np.zeros(3, dtype=float)
        self.normal_vec = np.zeros(3, dtype=float)
        self.force_vec = np.zeros(3, dtype=float)
        self.vel_vec = np.zeros(3, dtype=float)

        self.create_subscription(PoseStamped, "/crazyflie/out/EE_pose", self.cb_ee_pose, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/out/EE_velocity", self.cb_ee_vel, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/out/acc", self.cb_drone_acc, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/out/EE_acceleration", self.cb_ee_acc, 10)
        self.create_subscription(QuaternionStamped, "/estimated_contact_frame_quat", self.cb_contact_quat, 10)
        self.create_subscription(WrenchStamped, "/crazyflie/out/EE_contact_force_filt", self.cb_force, 10)
        self.create_subscription(Float32, "su/cmd_force", self.cb_force_cmd, 10)
        self.create_subscription(Float32MultiArray, "/crazyflie/out/motor_thrust", self.cb_pwm, 10)

        self.timer = self.create_timer(1.0 / self.update_hz, self.log_snapshot)

    def cb_ee_pose(self, msg: PoseStamped) -> None:
        with self.lock:
            self.ee_pos[:] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def cb_ee_vel(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.vel_vec[:] = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_drone_acc(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest_scalar["drone_acc_x"] = msg.vector.x
            self.latest_scalar["drone_acc_y"] = msg.vector.y
            self.latest_scalar["drone_acc_z"] = msg.vector.z

    def cb_ee_acc(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest_scalar["ee_acc_x"] = msg.vector.x
            self.latest_scalar["ee_acc_y"] = msg.vector.y
            self.latest_scalar["ee_acc_z"] = msg.vector.z

    def cb_contact_quat(self, msg: QuaternionStamped) -> None:
        q = msg.quaternion
        R = quat_xyzw_to_rotmat(q.x, q.y, q.z, q.w)
        n_world = R @ np.array([1.0, 0.0, 0.0], dtype=float)
        with self.lock:
            self.normal_vec[:] = n_world

    def cb_force(self, msg: WrenchStamped) -> None:
        fx = float(msg.wrench.force.x)
        fy = float(msg.wrench.force.y)
        fz = float(msg.wrench.force.z)
        with self.lock:
            self.force_vec[:] = [fx, fy, fz]
            self.latest_scalar["force_act"] = float(np.linalg.norm(self.force_vec))

    def cb_force_cmd(self, msg: Float32) -> None:
        with self.lock:
            self.latest_scalar["force_cmd"] = float(msg.data)

    def cb_pwm(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        with self.lock:
            self.latest_scalar["pwm1"] = float(msg.data[0])
            self.latest_scalar["pwm2"] = float(msg.data[1])
            self.latest_scalar["pwm3"] = float(msg.data[2])
            self.latest_scalar["pwm4"] = float(msg.data[3])

    def log_snapshot(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        with self.lock:
            self.data["t"].append(t)
            for key in self.latest_scalar:
                self.data[key].append(self.latest_scalar[key])

    def get_arrays(self) -> Dict[str, np.ndarray]:
        with self.lock:
            return {k: np.array(v, dtype=float) for k, v in self.data.items()}

    def get_vectors(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with self.lock:
            return (
                self.ee_pos.copy(),
                self.normal_vec.copy(),
                self.force_vec.copy(),
                self.vel_vec.copy(),
            )


class PlotWindow(QMainWindow):
    def __init__(self, rosbuf: ROSDataBuffer):
        super().__init__()
        self.rosbuf = rosbuf
        self.setWindowTitle("data_logging_arrow_pwm")

        pg.setConfigOptions(antialias=True)

        # fixed XY window size around EE position
        self.vec_half_width = 0.30
        self.vec_half_height = 0.30

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        self.info_label = QLabel("Waiting for data...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        root.addWidget(self.info_label)

        root.addWidget(self._build_vector_panel(), stretch=0, alignment=Qt.AlignHCenter)
        root.addWidget(self._build_force_panel(), stretch=0)
        root.addWidget(self._build_pwm_panel(), stretch=0)
        root.addWidget(self._build_acc_panel(), stretch=1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(100)

        # 더 좁게
        self.resize(680, 860)

    def _make_group_frame(self, title: str) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setLineWidth(1)

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("""
            QLabel {
                font-size: 15px;
                font-weight: bold;
                padding: 2px;
            }
        """)
        layout.addWidget(label)
        return frame

    def _build_vector_panel(self) -> QWidget:
        frame = self._make_group_frame("Vector Visualization (XY Top View)")
        layout = frame.layout()

        self.vec_plot = pg.PlotWidget()
        # 진짜 정사각형 위젯
        self.vec_plot.setFixedSize(300, 300)
        self.vec_plot.setBackground((55, 55, 55))
        self.vec_plot.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        pi = self.vec_plot.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("left", "Y [m]")
        pi.setLabel("bottom", "X [m]")
        pi.getAxis("left").setPen(pg.mkPen((235, 235, 235)))
        pi.getAxis("bottom").setPen(pg.mkPen((235, 235, 235)))
        pi.getAxis("left").setTextPen(pg.mkPen((235, 235, 235)))
        pi.getAxis("bottom").setTextPen(pg.mkPen((235, 235, 235)))
        pi.setAspectLocked(True)

        pi.enableAutoRange(axis="x", enable=False)
        pi.enableAutoRange(axis="y", enable=False)

        self.ee_point = pg.ScatterPlotItem(
            size=12,
            brush=pg.mkBrush(245, 245, 245),
            pen=pg.mkPen(15, 15, 15, width=1.5),
        )
        self.vec_plot.addItem(self.ee_point)

        self.normal_line = pg.PlotDataItem(pen=pg.mkPen((70, 130, 255), width=3))
        self.force_line = pg.PlotDataItem(pen=pg.mkPen((255, 90, 90), width=3))
        self.vel_line = pg.PlotDataItem(pen=pg.mkPen((80, 220, 120), width=3))

        self.normal_arrow = pg.ArrowItem(
            angle=180, headLen=14, tipAngle=30, baseAngle=20,
            brush=pg.mkBrush(70, 130, 255), pen=pg.mkPen((70, 130, 255), width=2)
        )
        self.force_arrow = pg.ArrowItem(
            angle=180, headLen=14, tipAngle=30, baseAngle=20,
            brush=pg.mkBrush(255, 90, 90), pen=pg.mkPen((255, 90, 90), width=2)
        )
        self.vel_arrow = pg.ArrowItem(
            angle=180, headLen=14, tipAngle=30, baseAngle=20,
            brush=pg.mkBrush(80, 220, 120), pen=pg.mkPen((80, 220, 120), width=2)
        )

        self.vec_plot.addItem(self.normal_line)
        self.vec_plot.addItem(self.force_line)
        self.vec_plot.addItem(self.vel_line)
        self.vec_plot.addItem(self.normal_arrow)
        self.vec_plot.addItem(self.force_arrow)
        self.vec_plot.addItem(self.vel_arrow)

        legend = QLabel("blue: normal(unit)    red: force(unit)    green: velocity(raw)")
        legend.setAlignment(Qt.AlignCenter)
        legend.setStyleSheet("font-size: 12px;")
        layout.addWidget(self.vec_plot, alignment=Qt.AlignHCenter)
        layout.addWidget(legend)
        return frame

    def _build_force_panel(self) -> QWidget:
        frame = self._make_group_frame("Force cmd vs. actual")
        layout = frame.layout()

        self.force_plot = pg.PlotWidget()
        self.force_plot.setBackground("w")
        self.force_plot.setMinimumHeight(110)

        pi = self.force_plot.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("left", "Force [N]")
        pi.setLabel("bottom", "time [s]")
        pi.getAxis("left").setPen(pg.mkPen("k"))
        pi.getAxis("bottom").setPen(pg.mkPen("k"))
        pi.getAxis("left").setTextPen(pg.mkPen("k"))
        pi.getAxis("bottom").setTextPen(pg.mkPen("k"))
        pi.setYRange(0.0, 0.1, padding=0.0)

        legend = pi.addLegend()
        legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))

        self.force_cmd_curve = self.force_plot.plot(name="force_cmd", pen=pg.mkPen((40, 90, 220), width=2))
        self.force_act_curve = self.force_plot.plot(name="force_act", pen=pg.mkPen((220, 50, 50), width=2))

        layout.addWidget(self.force_plot)
        return frame

    def _build_pwm_panel(self) -> QWidget:
        frame = self._make_group_frame("PWM 1 2 3 4")
        layout = frame.layout()

        self.pwm_plot = pg.PlotWidget()
        self.pwm_plot.setBackground("w")
        self.pwm_plot.setMinimumHeight(110)

        pi = self.pwm_plot.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("left", "Motor thrust")
        pi.setLabel("bottom", "time [s]")
        pi.getAxis("left").setPen(pg.mkPen("k"))
        pi.getAxis("bottom").setPen(pg.mkPen("k"))
        pi.getAxis("left").setTextPen(pg.mkPen("k"))
        pi.getAxis("bottom").setTextPen(pg.mkPen("k"))
        pi.setYRange(0.0, 0.2, padding=0.0)

        legend = pi.addLegend()
        legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))

        self.pwm1_curve = self.pwm_plot.plot(name="pwm1", pen=pg.mkPen((220, 50, 50), width=2))
        self.pwm2_curve = self.pwm_plot.plot(name="pwm2", pen=pg.mkPen((50, 160, 70), width=2))
        self.pwm3_curve = self.pwm_plot.plot(name="pwm3", pen=pg.mkPen((40, 90, 220), width=2))
        self.pwm4_curve = self.pwm_plot.plot(name="pwm4", pen=pg.mkPen((180, 120, 40), width=2))

        layout.addWidget(self.pwm_plot)
        return frame

    def _build_acc_panel(self) -> QWidget:
        frame = self._make_group_frame("Drone vs. EE Acceleration")
        outer = frame.layout()

        inner = QWidget()
        grid = QGridLayout(inner)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        def make_plot(title: str, yrange=(-0.1, 0.1)):
            w = pg.PlotWidget()
            w.setBackground("w")
            w.setMinimumHeight(150)
            pi = w.getPlotItem()
            pi.setTitle(title)
            pi.showGrid(x=True, y=True, alpha=0.25)
            pi.setLabel("left", "[m/s²]")
            pi.setLabel("bottom", "time [s]")
            pi.getAxis("left").setPen(pg.mkPen("k"))
            pi.getAxis("bottom").setPen(pg.mkPen("k"))
            pi.getAxis("left").setTextPen(pg.mkPen("k"))
            pi.getAxis("bottom").setTextPen(pg.mkPen("k"))
            pi.setYRange(yrange[0], yrange[1], padding=0.0)

            legend = pi.addLegend()
            legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))
            c1 = w.plot(name="drone", pen=pg.mkPen((220, 50, 50), width=2))
            c2 = w.plot(name="ee", pen=pg.mkPen((40, 90, 220), width=2))
            return w, c1, c2

        self.acc_x_plot, self.acc_x_drone_curve, self.acc_x_ee_curve = make_plot("Acceleration X")
        self.acc_y_plot, self.acc_y_drone_curve, self.acc_y_ee_curve = make_plot("Acceleration Y")
        self.acc_z_plot, self.acc_z_drone_curve, self.acc_z_ee_curve = make_plot("Acceleration Z")

        grid.addWidget(self.acc_x_plot, 0, 0)
        grid.addWidget(self.acc_y_plot, 0, 1)
        grid.addWidget(self.acc_z_plot, 0, 2)

        outer.addWidget(inner)
        return frame

    def _set_2d_vector(self, line_item, arrow_item, origin_xy: np.ndarray, vec_xy: np.ndarray, normalize: bool) -> None:
        vec_xy = np.array(vec_xy, dtype=float)
        if normalize:
            vec_xy = safe_unit(vec_xy)

        mag = float(np.linalg.norm(vec_xy))
        if mag < 1e-9:
            line_item.setData([], [])
            arrow_item.setStyle(headLen=0)
            arrow_item.setPos(origin_xy[0], origin_xy[1])
            return

        end_xy = origin_xy + vec_xy
        line_item.setData(
            [origin_xy[0], end_xy[0]],
            [origin_xy[1], end_xy[1]]
        )

        angle_deg = math.degrees(math.atan2(vec_xy[1], vec_xy[0]))
        arrow_item.setStyle(headLen=14, tipAngle=30, baseAngle=20)
        arrow_item.setPos(end_xy[0], end_xy[1])
        arrow_item.setRotation(angle_deg - 180.0)

    def update_all(self) -> None:
        arr = self.rosbuf.get_arrays()
        ee_pos, normal_vec, force_vec, vel_vec = self.rosbuf.get_vectors()

        ee_xy = ee_pos[:2]
        normal_xy = normal_vec[:2]
        force_xy = force_vec[:2]
        vel_xy = vel_vec[:2]

        self.ee_point.setData([{"pos": (ee_xy[0], ee_xy[1])}])

        # normal / force = unit length, velocity = raw
        self._set_2d_vector(self.normal_line, self.normal_arrow, ee_xy, normal_xy, normalize=True)
        self._set_2d_vector(self.force_line, self.force_arrow, ee_xy, force_xy, normalize=True)
        self._set_2d_vector(self.vel_line, self.vel_arrow, ee_xy, vel_xy, normalize=False)

        self.vec_plot.setXRange(
            ee_xy[0] - self.vec_half_width,
            ee_xy[0] + self.vec_half_width,
            padding=0.0,
        )
        self.vec_plot.setYRange(
            ee_xy[1] - self.vec_half_height,
            ee_xy[1] + self.vec_half_height,
            padding=0.0,
        )

        t = arr["t"]
        if t.size > 0:
            tmax = float(t[-1])
            window = float(self.rosbuf.history_sec)
            tmin = max(0.0, tmax - window)
            mask = t >= tmin
            t_win = t[mask]

            if t_win.size > 0:
                self.force_cmd_curve.setData(t_win, arr["force_cmd"][mask])
                self.force_act_curve.setData(t_win, arr["force_act"][mask])

                self.pwm1_curve.setData(t_win, arr["pwm1"][mask])
                self.pwm2_curve.setData(t_win, arr["pwm2"][mask])
                self.pwm3_curve.setData(t_win, arr["pwm3"][mask])
                self.pwm4_curve.setData(t_win, arr["pwm4"][mask])

                self.acc_x_drone_curve.setData(t_win, arr["drone_acc_x"][mask])
                self.acc_x_ee_curve.setData(t_win, arr["ee_acc_x"][mask])

                self.acc_y_drone_curve.setData(t_win, arr["drone_acc_y"][mask])
                self.acc_y_ee_curve.setData(t_win, arr["ee_acc_y"][mask])

                self.acc_z_drone_curve.setData(t_win, arr["drone_acc_z"][mask])
                self.acc_z_ee_curve.setData(t_win, arr["ee_acc_z"][mask])

                x_left = max(0.0, tmax - window)
                x_right = tmax if tmax >= window else window

                self.force_plot.setXRange(x_left, x_right, padding=0.0)
                self.pwm_plot.setXRange(x_left, x_right, padding=0.0)
                self.acc_x_plot.setXRange(x_left, x_right, padding=0.0)
                self.acc_y_plot.setXRange(x_left, x_right, padding=0.0)
                self.acc_z_plot.setXRange(x_left, x_right, padding=0.0)

        self.info_label.setText(
            f"EE=({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})   "
            f"normal=({normal_vec[0]:.3f}, {normal_vec[1]:.3f}, {normal_vec[2]:.3f})   "
            f"force=({force_vec[0]:.3f}, {force_vec[1]:.3f}, {force_vec[2]:.3f})   "
            f"vel=({vel_vec[0]:.3f}, {vel_vec[1]:.3f}, {vel_vec[2]:.3f})"
        )


def main() -> int:
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    app.setStyleSheet("""
    QWidget { background: #ffffff; color: #111111; }
    QMainWindow { background: #ffffff; }
    QLabel {
        font-family: monospace;
        font-size: 12px;
        padding: 2px 4px 2px 4px;
    }
    """)

    rclpy.init()
    rosbuf = ROSDataBuffer()

    ros_thread = threading.Thread(target=rclpy.spin, args=(rosbuf,), daemon=True)
    ros_thread.start()

    win = PlotWindow(rosbuf)
    win.show()

    def _sigint_handler(*_args):
        app.quit()

    signal.signal(signal.SIGINT, _sigint_handler)

    ret = app.exec_()

    rosbuf.destroy_node()
    rclpy.shutdown()
    return int(ret)


if __name__ == "__main__":
    raise SystemExit(main())