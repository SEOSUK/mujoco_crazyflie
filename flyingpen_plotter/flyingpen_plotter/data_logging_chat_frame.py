#!/usr/bin/env python3
from __future__ import annotations

import signal
import sys
import threading
from collections import deque
from typing import Dict

import numpy as np

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QGridLayout, QLabel, QMainWindow, QVBoxLayout, QWidget, QFrame

import pyqtgraph as pg

from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from std_msgs.msg import Float32, Float32MultiArray


def quat_xyzw_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
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
        super().__init__("data_logging_chat_frame")

        self.declare_parameter("history_sec", 8.0)
        self.declare_parameter("update_hz", 50.0)

        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(200, int(self.history_sec * self.update_hz) + 50)

        keys = [
            "vel_y_cmd", "vel_y_act",
            "vel_z_cmd", "vel_z_act",
            "force_x_cmd", "force_x_act",
            "thrust1", "thrust2", "thrust3", "thrust4",
        ]

        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for k in keys:
            self.data[k] = deque(maxlen=self.maxlen)

        self.latest_scalar = {k: np.nan for k in keys}
        self.contact_R_C = np.eye(3, dtype=float)

        self.create_subscription(QuaternionStamped, "/estimated_contact_frame_quat", self.cb_contact_quat, 10)
        self.create_subscription(Vector3Stamped, "/su/debug/contact_vel_cmd", self.cb_contact_vel_cmd, 10)
        self.create_subscription(Vector3Stamped, "/su/debug/contact_vel_actual", self.cb_contact_vel_actual, 10)
        self.create_subscription(Float32, "su/cmd_force", self.cb_force_cmd, 10)
        self.create_subscription(Float32, "/su/contact_force_x", self.cb_force_x_actual, 10)
        self.create_subscription(Float32MultiArray, "/crazyflie/out/motor_thrust", self.cb_motor_thrust, 10)

        self.timer = self.create_timer(1.0 / self.update_hz, self.log_snapshot)

    def cb_contact_quat(self, msg: QuaternionStamped) -> None:
        q = msg.quaternion
        R_C = quat_xyzw_to_rotmat(q.x, q.y, q.z, q.w)
        with self.lock:
            self.contact_R_C[:, :] = R_C

    def cb_contact_vel_cmd(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest_scalar["vel_y_cmd"] = float(msg.vector.y)
            self.latest_scalar["vel_z_cmd"] = float(msg.vector.z)

    def cb_contact_vel_actual(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest_scalar["vel_y_act"] = float(msg.vector.y)
            self.latest_scalar["vel_z_act"] = float(msg.vector.z)

    def cb_force_cmd(self, msg: Float32) -> None:
        with self.lock:
            self.latest_scalar["force_x_cmd"] = float(msg.data)

    def cb_force_x_actual(self, msg: Float32) -> None:
        with self.lock:
            self.latest_scalar["force_x_act"] = float(msg.data)

    def cb_motor_thrust(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        with self.lock:
            self.latest_scalar["thrust1"] = float(msg.data[0])
            self.latest_scalar["thrust2"] = float(msg.data[1])
            self.latest_scalar["thrust3"] = float(msg.data[2])
            self.latest_scalar["thrust4"] = float(msg.data[3])

    def log_snapshot(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        with self.lock:
            self.data["t"].append(t)
            for key in self.latest_scalar:
                self.data[key].append(self.latest_scalar[key])

    def get_arrays(self) -> Dict[str, np.ndarray]:
        with self.lock:
            return {k: np.array(v, dtype=float) for k, v in self.data.items()}


class PlotWindow(QMainWindow):
    def __init__(self, rosbuf: ROSDataBuffer):
        super().__init__()
        self.rosbuf = rosbuf
        self.setWindowTitle("data_logging_chat_frame")

        pg.setConfigOptions(antialias=True)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        self.info_label = QLabel("contact frame: y,z velocity and x force")
        root.addWidget(self.info_label)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        root.addLayout(grid)

        self.vel_y_plot, self.vel_y_cmd_curve, self.vel_y_act_curve = self.make_compare_plot(
            "Contact velocity Y", "vel [m/s]"
        )
        self.vel_z_plot, self.vel_z_cmd_curve, self.vel_z_act_curve = self.make_compare_plot(
            "Contact velocity Z", "vel [m/s]"
        )
        self.force_x_plot, self.force_x_cmd_curve, self.force_x_act_curve = self.make_compare_plot(
            "Contact force X", "force [N]"
        )

        self.thrust_plot, self.thrust1_curve, self.thrust2_curve, self.thrust3_curve, self.thrust4_curve = self.make_thrust_plot()

        grid.addWidget(self.vel_y_plot, 0, 0)
        grid.addWidget(self.vel_z_plot, 0, 1)
        grid.addWidget(self.force_x_plot, 0, 2)
        grid.addWidget(self.thrust_plot, 1, 0, 1, 3)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(100)

        self.resize(1300, 700)

    def make_group_frame(self, title: str) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        label = QLabel(title)
        label.setStyleSheet("font-size: 15px; font-weight: bold;")
        layout.addWidget(label)
        return frame

    def make_compare_plot(self, title: str, ylabel: str):
        frame = self.make_group_frame(title)
        layout = frame.layout()

        plot = pg.PlotWidget()
        plot.setBackground("w")
        plot.setMinimumHeight(240)
        pi = plot.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("left", ylabel)
        pi.setLabel("bottom", "time [s]")
        pi.getAxis("left").setPen(pg.mkPen("k"))
        pi.getAxis("bottom").setPen(pg.mkPen("k"))
        pi.getAxis("left").setTextPen(pg.mkPen("k"))
        pi.getAxis("bottom").setTextPen(pg.mkPen("k"))
        legend = pi.addLegend()
        legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))
        cmd_curve = plot.plot(name="desired", pen=pg.mkPen((40, 90, 220), width=2))
        act_curve = plot.plot(name="actual", pen=pg.mkPen((220, 50, 50), width=2))
        layout.addWidget(plot)
        return frame, cmd_curve, act_curve

    def make_thrust_plot(self):
        frame = self.make_group_frame("Motor thrust 1 2 3 4")
        layout = frame.layout()

        plot = pg.PlotWidget()
        plot.setBackground("w")
        plot.setMinimumHeight(280)
        pi = plot.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("left", "thrust")
        pi.setLabel("bottom", "time [s]")
        pi.getAxis("left").setPen(pg.mkPen("k"))
        pi.getAxis("bottom").setPen(pg.mkPen("k"))
        pi.getAxis("left").setTextPen(pg.mkPen("k"))
        pi.getAxis("bottom").setTextPen(pg.mkPen("k"))
        legend = pi.addLegend()
        legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))

        c1 = plot.plot(name="thrust1", pen=pg.mkPen((220, 50, 50), width=2))
        c2 = plot.plot(name="thrust2", pen=pg.mkPen((50, 160, 70), width=2))
        c3 = plot.plot(name="thrust3", pen=pg.mkPen((40, 90, 220), width=2))
        c4 = plot.plot(name="thrust4", pen=pg.mkPen((180, 120, 40), width=2))
        layout.addWidget(plot)
        return frame, c1, c2, c3, c4

    def update_all(self) -> None:
        arr = self.rosbuf.get_arrays()
        t = arr["t"]
        if t.size == 0:
            return

        tmax = float(t[-1])
        window = float(self.rosbuf.history_sec)
        tmin = max(0.0, tmax - window)
        mask = t >= tmin
        t_win = t[mask]
        if t_win.size == 0:
            return

        self.vel_y_cmd_curve.setData(t_win, arr["vel_y_cmd"][mask])
        self.vel_y_act_curve.setData(t_win, arr["vel_y_act"][mask])

        self.vel_z_cmd_curve.setData(t_win, arr["vel_z_cmd"][mask])
        self.vel_z_act_curve.setData(t_win, arr["vel_z_act"][mask])

        self.force_x_cmd_curve.setData(t_win, arr["force_x_cmd"][mask])
        self.force_x_act_curve.setData(t_win, arr["force_x_act"][mask])

        self.thrust1_curve.setData(t_win, arr["thrust1"][mask])
        self.thrust2_curve.setData(t_win, arr["thrust2"][mask])
        self.thrust3_curve.setData(t_win, arr["thrust3"][mask])
        self.thrust4_curve.setData(t_win, arr["thrust4"][mask])

        x_left = max(0.0, tmax - window)
        x_right = tmax if tmax >= window else window

        for widget in [self.vel_y_plot, self.vel_z_plot, self.force_x_plot, self.thrust_plot]:
            widget.layout().itemAt(1).widget().setXRange(x_left, x_right, padding=0.0)

        self.info_label.setText(
            f"latest | vy_cmd={arr['vel_y_cmd'][-1]:.3f}, vy_act={arr['vel_y_act'][-1]:.3f}, "
            f"vz_cmd={arr['vel_z_cmd'][-1]:.3f}, vz_act={arr['vel_z_act'][-1]:.3f}, "
            f"fx_cmd={arr['force_x_cmd'][-1]:.3f}, fx_act={arr['force_x_act'][-1]:.3f}"
        )


def main() -> int:
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    app.setStyleSheet(
        """
        QWidget { background: #ffffff; color: #111111; }
        QMainWindow { background: #ffffff; }
        QLabel { font-family: monospace; font-size: 12px; padding: 2px 4px 2px 4px; }
        """
    )

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