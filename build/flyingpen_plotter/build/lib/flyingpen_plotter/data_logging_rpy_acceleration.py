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
    QVBoxLayout,
    QWidget,
    QFrame,
    QGridLayout,
)

import pyqtgraph as pg

from geometry_msgs.msg import PoseStamped, Vector3Stamped


def quat_to_rpy_xyz(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def unwrap_angle(prev: float, cur: float) -> float:
    d = cur - prev
    while d > math.pi:
        cur -= 2.0 * math.pi
        d = cur - prev
    while d < -math.pi:
        cur += 2.0 * math.pi
        d = cur - prev
    return cur


def lpf_scalar(x: float, prev: float, dt: float, cutoff_hz: float) -> float:
    if cutoff_hz <= 1e-9:
        return x
    wc = 2.0 * math.pi * cutoff_hz
    alpha = 1.0 - math.exp(-wc * dt)
    return (1.0 - alpha) * prev + alpha * x


class ROSDataBuffer(Node):
    def __init__(self):
        super().__init__("data_logging_rpy_acceleration")

        self.declare_parameter("history_sec", 5.0)
        self.declare_parameter("update_hz", 30.0)

        # desired RPY LPF
        self.declare_parameter("rpy_des_lpf.enable", True)
        self.declare_parameter("rpy_des_lpf.cutoff_hz", 5.0)

        # ee_corr = ee_raw + ee_bias
        # dot(ee_bias) = -Ki * (ee_corr - drone) - leak * ee_bias
        self.declare_parameter("ee_bias_integrator.enable", True)
        self.declare_parameter("ee_bias_integrator.ki", 0.1)          # [1/s]
        self.declare_parameter("ee_bias_integrator.leak", 0.0)        # [1/s]
        self.declare_parameter("ee_bias_integrator.max_bias", 2.0)    # [m/s^2]
        self.declare_parameter("ee_bias_integrator.use_raw_init", False)

        self.topic_pose_actual = "/crazyflie/out/pose"
        self.topic_rpy_desired = "/crazyflie/debug/rpy_des"
        self.topic_drone_acc = "/crazyflie/out/acc"
        self.topic_ee_acc = "/crazyflie/out/EE_acceleration"

        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)

        self.rpy_des_lpf_enable = bool(self.get_parameter("rpy_des_lpf.enable").value)
        self.rpy_des_lpf_cutoff_hz = float(self.get_parameter("rpy_des_lpf.cutoff_hz").value)

        self.ee_bias_enable = bool(self.get_parameter("ee_bias_integrator.enable").value)
        self.ee_bias_ki = float(self.get_parameter("ee_bias_integrator.ki").value)
        self.ee_bias_leak = float(self.get_parameter("ee_bias_integrator.leak").value)
        self.ee_bias_max = float(self.get_parameter("ee_bias_integrator.max_bias").value)
        self.ee_bias_use_raw_init = bool(self.get_parameter("ee_bias_integrator.use_raw_init").value)

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(100, int(self.history_sec * self.update_hz) + 20)

        keys = [
            "roll_act", "pitch_act", "yaw_act",
            "roll_des", "pitch_des", "yaw_des",
            "drone_acc_x", "drone_acc_y", "drone_acc_z",
            "ee_acc_x", "ee_acc_y", "ee_acc_z",
            "ee_bias_x", "ee_bias_y", "ee_bias_z",
            "ee_gap_x", "ee_gap_y", "ee_gap_z",
        ]

        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for k in keys:
            self.data[k] = deque(maxlen=self.maxlen)

        self.latest_scalar = {k: np.nan for k in keys}

        self.prev_yaw_act = None
        self.prev_yaw_des = None

        self.latest_drone_acc = np.zeros(3, dtype=float)
        self.latest_ee_acc_raw = np.zeros(3, dtype=float)
        self.ee_bias = np.zeros(3, dtype=float)

        self.bias_initialized = False
        self.prev_snapshot_time = None

        self.rpy_des_lpf_initialized = False
        self.rpy_des_prev_time = None
        self.roll_des_filt = 0.0
        self.pitch_des_filt = 0.0
        self.yaw_des_filt = 0.0

        self.create_subscription(PoseStamped, self.topic_pose_actual, self.cb_pose_actual, 10)
        self.create_subscription(Vector3Stamped, self.topic_rpy_desired, self.cb_rpy_desired, 10)
        self.create_subscription(Vector3Stamped, self.topic_drone_acc, self.cb_drone_acc, 10)
        self.create_subscription(Vector3Stamped, self.topic_ee_acc, self.cb_ee_acc, 10)

        self.timer = self.create_timer(1.0 / self.update_hz, self.log_snapshot)

    def cb_pose_actual(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        roll, pitch, yaw = quat_to_rpy_xyz(q.x, q.y, q.z, q.w)

        with self.lock:
            if self.prev_yaw_act is None:
                yaw_use = yaw
            else:
                yaw_use = unwrap_angle(self.prev_yaw_act, yaw)

            self.prev_yaw_act = yaw_use
            self.latest_scalar["roll_act"] = roll
            self.latest_scalar["pitch_act"] = pitch
            self.latest_scalar["yaw_act"] = yaw_use

    def cb_rpy_desired(self, msg: Vector3Stamped) -> None:
        roll = float(msg.vector.x)
        pitch = float(msg.vector.y)
        yaw = float(msg.vector.z)

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        with self.lock:
            if self.prev_yaw_des is None:
                yaw_unwrapped = yaw
            else:
                yaw_unwrapped = unwrap_angle(self.prev_yaw_des, yaw)
            self.prev_yaw_des = yaw_unwrapped

            if not self.rpy_des_lpf_initialized:
                self.roll_des_filt = roll
                self.pitch_des_filt = pitch
                self.yaw_des_filt = yaw_unwrapped
                self.rpy_des_lpf_initialized = True
                dt = 1.0 / max(self.update_hz, 1e-6)
            else:
                if self.rpy_des_prev_time is None:
                    dt = 1.0 / max(self.update_hz, 1e-6)
                else:
                    dt = max(1e-6, now_sec - self.rpy_des_prev_time)

                if self.rpy_des_lpf_enable:
                    self.roll_des_filt = lpf_scalar(
                        roll, self.roll_des_filt, dt, self.rpy_des_lpf_cutoff_hz
                    )
                    self.pitch_des_filt = lpf_scalar(
                        pitch, self.pitch_des_filt, dt, self.rpy_des_lpf_cutoff_hz
                    )
                    self.yaw_des_filt = lpf_scalar(
                        yaw_unwrapped, self.yaw_des_filt, dt, self.rpy_des_lpf_cutoff_hz
                    )
                else:
                    self.roll_des_filt = roll
                    self.pitch_des_filt = pitch
                    self.yaw_des_filt = yaw_unwrapped

            self.rpy_des_prev_time = now_sec

            self.latest_scalar["roll_des"] = self.roll_des_filt
            self.latest_scalar["pitch_des"] = self.pitch_des_filt
            self.latest_scalar["yaw_des"] = self.yaw_des_filt

    def cb_drone_acc(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest_drone_acc[:] = [float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)]
            self.latest_scalar["drone_acc_x"] = self.latest_drone_acc[0]
            self.latest_scalar["drone_acc_y"] = self.latest_drone_acc[1]
            self.latest_scalar["drone_acc_z"] = self.latest_drone_acc[2]

    def cb_ee_acc(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest_ee_acc_raw[:] = [float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)]

            if (not self.bias_initialized) and self.ee_bias_use_raw_init:
                self.ee_bias[:] = self.latest_drone_acc - self.latest_ee_acc_raw
                self.bias_initialized = True

    def log_snapshot(self) -> None:
        t_now = self.get_clock().now().nanoseconds * 1e-9
        t = t_now - self.t0

        with self.lock:
            if self.prev_snapshot_time is None:
                dt = 1.0 / max(self.update_hz, 1e-6)
            else:
                dt = max(1e-6, t_now - self.prev_snapshot_time)
            self.prev_snapshot_time = t_now

            if not self.bias_initialized:
                self.ee_bias[:] = 0.0
                self.bias_initialized = True

            if self.ee_bias_enable:
                ee_acc_corr = self.latest_ee_acc_raw + self.ee_bias
                err = ee_acc_corr - self.latest_drone_acc
                self.ee_bias += dt * (-self.ee_bias_ki * err - self.ee_bias_leak * self.ee_bias)
                self.ee_bias = np.clip(self.ee_bias, -self.ee_bias_max, self.ee_bias_max)
                ee_acc_corr = self.latest_ee_acc_raw + self.ee_bias
            else:
                self.ee_bias[:] = 0.0
                ee_acc_corr = self.latest_ee_acc_raw.copy()

            gap = ee_acc_corr - self.latest_drone_acc

            self.latest_scalar["ee_acc_x"] = ee_acc_corr[0]
            self.latest_scalar["ee_acc_y"] = ee_acc_corr[1]
            self.latest_scalar["ee_acc_z"] = ee_acc_corr[2]

            self.latest_scalar["ee_bias_x"] = self.ee_bias[0]
            self.latest_scalar["ee_bias_y"] = self.ee_bias[1]
            self.latest_scalar["ee_bias_z"] = self.ee_bias[2]

            self.latest_scalar["ee_gap_x"] = gap[0]
            self.latest_scalar["ee_gap_y"] = gap[1]
            self.latest_scalar["ee_gap_z"] = gap[2]

            self.data["t"].append(t)
            for key in self.latest_scalar:
                self.data[key].append(self.latest_scalar[key])

    def get_arrays(self) -> Dict[str, np.ndarray]:
        with self.lock:
            return {k: np.array(v, dtype=float) for k, v in self.data.items()}

    def get_latest_text(self) -> str:
        with self.lock:
            return (
                f"RPY act=({self.latest_scalar['roll_act']:.3f}, "
                f"{self.latest_scalar['pitch_act']:.3f}, "
                f"{self.latest_scalar['yaw_act']:.3f})   "
                f"RPY des=({self.latest_scalar['roll_des']:.3f}, "
                f"{self.latest_scalar['pitch_des']:.3f}, "
                f"{self.latest_scalar['yaw_des']:.3f})   "
                f"bias=({self.latest_scalar['ee_bias_x']:.3f}, "
                f"{self.latest_scalar['ee_bias_y']:.3f}, "
                f"{self.latest_scalar['ee_bias_z']:.3f})   "
                f"gap=({self.latest_scalar['ee_gap_x']:.3f}, "
                f"{self.latest_scalar['ee_gap_y']:.3f}, "
                f"{self.latest_scalar['ee_gap_z']:.3f})"
            )


class PlotWindow(QMainWindow):
    def __init__(self, rosbuf: ROSDataBuffer):
        super().__init__()
        self.rosbuf = rosbuf
        self.setWindowTitle("data_logging_rpy_acceleration")

        pg.setConfigOptions(antialias=True)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        self.info_label = QLabel("Waiting for data...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        root.addWidget(self.info_label)

        root.addWidget(self._build_rpy_panel(), stretch=1)
        root.addWidget(self._build_acc_panel(), stretch=1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(100)

        self.resize(980, 900)

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

    def _style_plot(self, plot_widget: pg.PlotWidget, y_label: str):
        plot_widget.setBackground("w")
        pi = plot_widget.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("left", y_label)
        pi.setLabel("bottom", "time [s]")
        pi.getAxis("left").setPen(pg.mkPen("k"))
        pi.getAxis("bottom").setPen(pg.mkPen("k"))
        pi.getAxis("left").setTextPen(pg.mkPen("k"))
        pi.getAxis("bottom").setTextPen(pg.mkPen("k"))

        legend = pi.addLegend()
        legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))

    def _build_rpy_panel(self) -> QWidget:
        frame = self._make_group_frame("Drone RPY: desired vs actual")
        outer = frame.layout()

        inner = QWidget()
        grid = QGridLayout(inner)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        def make_plot(title: str, y_range=(-0.1, 0.1)):
            w = pg.PlotWidget()
            w.setMinimumHeight(180)
            self._style_plot(w, "[rad]")
            pi = w.getPlotItem()
            pi.setTitle(title)
            pi.setYRange(y_range[0], y_range[1], padding=0.0)

            c_des = w.plot(name="desired", pen=pg.mkPen((40, 90, 220), width=2))
            c_act = w.plot(name="actual", pen=pg.mkPen((220, 50, 50), width=2))
            return w, c_des, c_act

        self.roll_plot, self.roll_des_curve, self.roll_act_curve = make_plot("Roll")
        self.pitch_plot, self.pitch_des_curve, self.pitch_act_curve = make_plot("Pitch")
        self.yaw_plot, self.yaw_des_curve, self.yaw_act_curve = make_plot("Yaw")

        grid.addWidget(self.roll_plot, 0, 0)
        grid.addWidget(self.pitch_plot, 0, 1)
        grid.addWidget(self.yaw_plot, 0, 2)

        outer.addWidget(inner)
        return frame

    def _build_acc_panel(self) -> QWidget:
        frame = self._make_group_frame("Drone vs End Effector Acceleration")
        outer = frame.layout()

        inner = QWidget()
        grid = QGridLayout(inner)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        def make_plot(title: str, y_range=(-1.0, 1.0)):
            w = pg.PlotWidget()
            w.setMinimumHeight(180)
            self._style_plot(w, "[m/s²]")
            pi = w.getPlotItem()
            pi.setTitle(title)
            pi.setYRange(y_range[0], y_range[1], padding=0.0)

            c_drone = w.plot(name="drone", pen=pg.mkPen((220, 50, 50), width=2))
            c_ee_corr = w.plot(name="EE", pen=pg.mkPen((40, 90, 220), width=2))
            return w, c_drone, c_ee_corr

        self.acc_x_plot, self.acc_x_drone_curve, self.acc_x_ee_curve = make_plot("Acceleration X")
        self.acc_y_plot, self.acc_y_drone_curve, self.acc_y_ee_curve = make_plot("Acceleration Y")
        self.acc_z_plot, self.acc_z_drone_curve, self.acc_z_ee_curve = make_plot("Acceleration Z")

        grid.addWidget(self.acc_x_plot, 0, 0)
        grid.addWidget(self.acc_y_plot, 0, 1)
        grid.addWidget(self.acc_z_plot, 0, 2)

        outer.addWidget(inner)
        return frame

    def update_all(self) -> None:
        arr = self.rosbuf.get_arrays()
        t = arr["t"]

        self.info_label.setText(self.rosbuf.get_latest_text())

        if t.size == 0:
            return

        tmax = float(t[-1])
        window = float(self.rosbuf.history_sec)
        tmin = max(0.0, tmax - window)
        mask = t >= tmin
        t_win = t[mask]

        if t_win.size == 0:
            return

        self.roll_des_curve.setData(t_win, arr["roll_des"][mask])
        self.roll_act_curve.setData(t_win, arr["roll_act"][mask])

        self.pitch_des_curve.setData(t_win, arr["pitch_des"][mask])
        self.pitch_act_curve.setData(t_win, arr["pitch_act"][mask])

        self.yaw_des_curve.setData(t_win, arr["yaw_des"][mask])
        self.yaw_act_curve.setData(t_win, arr["yaw_act"][mask])

        self.acc_x_drone_curve.setData(t_win, arr["drone_acc_x"][mask])
        self.acc_x_ee_curve.setData(t_win, arr["ee_acc_x"][mask])

        self.acc_y_drone_curve.setData(t_win, arr["drone_acc_y"][mask])
        self.acc_y_ee_curve.setData(t_win, arr["ee_acc_y"][mask])

        self.acc_z_drone_curve.setData(t_win, arr["drone_acc_z"][mask])
        self.acc_z_ee_curve.setData(t_win, arr["ee_acc_z"][mask])

        x_left = max(0.0, tmax - window)
        x_right = tmax if tmax >= window else window

        for w in [
            self.roll_plot, self.pitch_plot, self.yaw_plot,
            self.acc_x_plot, self.acc_y_plot, self.acc_z_plot,
        ]:
            w.setXRange(x_left, x_right, padding=0.0)


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