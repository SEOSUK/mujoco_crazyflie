#!/usr/bin/env python3
from __future__ import annotations

import math
import signal
import sys
import threading
from collections import deque
from typing import Dict

import numpy as np

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QSizePolicy, QVBoxLayout, QWidget

import pyqtgraph as pg

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float32, Float32MultiArray


AXES = ("x", "y", "z")
X_HORIZON_SEC = 10.0
Y_LIMITS = {
    "x": (-0.01, 0.05),
    "y": (-0.03, 0.03),
    "z": (-0.03, 0.03),
}


def quat_xyzw_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1.0e-12:
        return np.eye(3, dtype=float)

    qx /= n
    qy /= n
    qz /= n
    qw /= n

    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
        [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)],
        [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=float)


def wrench_force(msg: WrenchStamped) -> np.ndarray:
    return np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], dtype=float)


def nan_vec() -> np.ndarray:
    return np.full(3, math.nan, dtype=float)


class ROSDataBuffer(Node):
    def __init__(self):
        super().__init__("mob_wrench_compare")

        self.declare_parameter("history_sec", 5.0)
        self.declare_parameter("update_hz", 60.0)
        self.declare_parameter("render_hz", 10.0)
        self.declare_parameter("pose_topic", "/crazyflie/out/pose")
        self.declare_parameter("input_topic", "/crazyflie/in/input")
        self.declare_parameter("cmd_force_topic", "su/cmd_force")
        self.declare_parameter("pure_topic", "/crazyflie/out/mob_2nd")
        self.declare_parameter("k_ep_topic", "/crazyflie/out/mob_2nd_tau")
        self.declare_parameter("k_epi_topic", "/crazyflie/out/mob_2nd_tau_i")
        self.declare_parameter("kalman_topic", "/crazyflie/out/mob_kalman")
        self.declare_parameter("adaptive_topic", "/crazyflie/out/mob_adaptive")
        self.declare_parameter("ee_force_topic", "/crazyflie/out/EE_contact_force")

        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = max(1.0, float(self.get_parameter("update_hz").value))
        self.render_hz = max(0.1, float(self.get_parameter("render_hz").value))
        self.buffer_sec = max(self.history_sec, X_HORIZON_SEC) + 1.0

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(120, int(self.buffer_sec * self.update_hz) + 20)

        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for prefix in ("pure", "k_ep", "k_epi", "kalman", "adaptive", "u_neg", "ee"):
            for axis in AXES:
                self.data[f"{prefix}_{axis}"] = deque(maxlen=self.maxlen)
        self.data["desired_x"] = deque(maxlen=self.maxlen)

        self.latest = {key: math.nan for key in self.data if key != "t"}
        self.rot_body_to_world = np.eye(3, dtype=float)
        self.input_tau_fz = np.full(4, math.nan, dtype=float)
        self.cmd_force_desired = math.nan
        self.pure_force = nan_vec()
        self.k_ep_force = nan_vec()
        self.k_epi_force = nan_vec()
        self.kalman_force = nan_vec()
        self.adaptive_force = nan_vec()
        self.ee_force = nan_vec()

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.cb_pose,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("input_topic").value),
            self.cb_input,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("cmd_force_topic").value),
            self.cb_cmd_force,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("pure_topic").value),
            self.cb_pure,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("k_ep_topic").value),
            self.cb_k_ep,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("k_epi_topic").value),
            self.cb_k_epi,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("kalman_topic").value),
            self.cb_kalman,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("adaptive_topic").value),
            self.cb_adaptive,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("ee_force_topic").value),
            self.cb_ee_force,
            10,
        )

        self.timer = self.create_timer(1.0 / self.update_hz, self.log_snapshot)
        self.get_logger().info("mob_wrench_compare started")

    def cb_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        rot = quat_xyzw_to_rotmat(q.x, q.y, q.z, q.w)
        with self.lock:
            self.rot_body_to_world = rot

    def cb_input(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        with self.lock:
            self.input_tau_fz = np.array(msg.data[:4], dtype=float)

    def cb_cmd_force(self, msg: Float32) -> None:
        with self.lock:
            self.cmd_force_desired = float(msg.data)

    def cb_pure(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.pure_force = wrench_force(msg)

    def cb_k_ep(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.k_ep_force = wrench_force(msg)

    def cb_k_epi(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.k_epi_force = wrench_force(msg)

    def cb_kalman(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.kalman_force = wrench_force(msg)

    def cb_adaptive(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.adaptive_force = wrench_force(msg)

    def cb_ee_force(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.ee_force = wrench_force(msg)

    def log_snapshot(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0

        with self.lock:
            input_tau_fz = self.input_tau_fz.copy()
            rot_body_to_world = self.rot_body_to_world.copy()
            desired_x = float(self.cmd_force_desired)
            pure_force = -self.pure_force.copy()
            k_ep_force = -self.k_ep_force.copy()
            k_epi_force = -self.k_epi_force.copy()
            kalman_force = -self.kalman_force.copy()
            adaptive_force = -self.adaptive_force.copy()
            ee_force = self.ee_force.copy()

            if np.all(np.isfinite(input_tau_fz)):
                control_force_world = rot_body_to_world @ np.array([0.0, 0.0, input_tau_fz[3]], dtype=float)
                u_input = control_force_world
            else:
                u_input = nan_vec()

            snapshot = {
                "pure": pure_force,
                "k_ep": k_ep_force,
                "k_epi": k_epi_force,
                "kalman": kalman_force,
                "adaptive": adaptive_force,
                "u_neg": u_input,
                "ee": ee_force,
            }

            self.data["t"].append(t)
            self.latest["desired_x"] = desired_x
            self.data["desired_x"].append(desired_x)
            for prefix, vec in snapshot.items():
                for axis_index, axis_name in enumerate(AXES):
                    value = float(vec[axis_index])
                    key = f"{prefix}_{axis_name}"
                    self.latest[key] = value
                    self.data[key].append(value)

    def get_arrays(self) -> Dict[str, np.ndarray]:
        with self.lock:
            return {key: np.array(values, dtype=float) for key, values in self.data.items()}

    def get_summary(self) -> Dict[str, float]:
        with self.lock:
            return dict(self.latest)


class PlotWindow(QMainWindow):
    def __init__(self, rosbuf: ROSDataBuffer):
        super().__init__()
        self.rosbuf = rosbuf
        self.setWindowTitle("mob_wrench_compare")

        pg.setConfigOptions(antialias=True)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        self.info_label = QLabel("Waiting for MOB / control-input traces...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.info_label.setWordWrap(True)
        self.info_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        root.addWidget(self.info_label)

        colors = {
            "pure": pg.mkPen((40, 90, 220, 220), width=1.8),
            "k_ep": pg.mkPen((220, 40, 40, 210), width=2.0, style=Qt.DashLine),
            "k_epi": pg.mkPen((140, 60, 220, 180), width=1.2),
            "kalman": pg.mkPen((20, 145, 55, 220), width=1.8),
            "adaptive": pg.mkPen((0, 150, 180, 220), width=1.6, style=Qt.DotLine),
            "u_neg": pg.mkPen((30, 30, 30, 180), width=1.5, style=Qt.DashLine),
            "ee": pg.mkPen((230, 140, 20, 220), width=2.0),
            "desired_x": pg.mkPen((20, 150, 90), width=3.0),
        }

        self.plots = []
        self.curves = {}
        for axis_name in AXES:
            plot = self._make_plot(
                axis_name,
                f"Force {axis_name}",
                f"{axis_name}-axis force [N]",
            )
            self.plots.append(plot)
            if axis_name == "x":
                self.curves["desired_x"] = plot.plot(name="desired x force", pen=colors["desired_x"])
            self.curves[f"pure_{axis_name}"] = plot.plot(name="pure", pen=colors["pure"])
            self.curves[f"k_ep_{axis_name}"] = plot.plot(name="k_ep", pen=colors["k_ep"])
            self.curves[f"k_epi_{axis_name}"] = plot.plot(
                name="k_epi",
                pen=colors["k_epi"],
                symbol="o",
                symbolSize=3,
                symbolBrush=(140, 60, 220, 90),
                symbolPen=pg.mkPen((140, 60, 220, 120), width=0.8),
            )
            self.curves[f"kalman_{axis_name}"] = plot.plot(name="kalman", pen=colors["kalman"])
            self.curves[f"adaptive_{axis_name}"] = plot.plot(name="adaptive", pen=colors["adaptive"])
            self.curves[f"u_neg_{axis_name}"] = plot.plot(name="control input", pen=colors["u_neg"])
            self.curves[f"ee_{axis_name}"] = plot.plot(name="ee_tip force", pen=colors["ee"])
            if axis_name == "x":
                self.curves["desired_x"].setZValue(10)
            self.curves[f"pure_{axis_name}"].setZValue(2)
            self.curves[f"u_neg_{axis_name}"].setZValue(3)
            self.curves[f"k_ep_{axis_name}"].setZValue(4)
            self.curves[f"k_epi_{axis_name}"].setZValue(5)
            self.curves[f"kalman_{axis_name}"].setZValue(6)
            self.curves[f"adaptive_{axis_name}"].setZValue(7)
            self.curves[f"ee_{axis_name}"].setZValue(8)
            root.addWidget(plot, stretch=1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(max(1, int(round(1000.0 / self.rosbuf.render_hz))))

        self.resize(920, 980)

    def _make_plot(self, axis_name: str, title: str, ylabel: str) -> pg.PlotWidget:
        plot = pg.PlotWidget()
        plot.setBackground("w")
        plot.setMinimumHeight(240)
        plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        plot.setMouseEnabled(x=True, y=True)
        pi = plot.getPlotItem()
        pi.setTitle(title)
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("left", ylabel)
        pi.setLabel("bottom", "time [s]")
        pi.getAxis("left").setPen(pg.mkPen("k"))
        pi.getAxis("bottom").setPen(pg.mkPen("k"))
        pi.getAxis("left").setTextPen(pg.mkPen("k"))
        pi.getAxis("bottom").setTextPen(pg.mkPen("k"))
        pi.enableAutoRange(axis="x", enable=False)
        pi.enableAutoRange(axis="y", enable=False)
        plot.setLimits(minXRange=X_HORIZON_SEC, maxXRange=X_HORIZON_SEC)
        plot.setXRange(0.0, X_HORIZON_SEC, padding=0.0)
        y_min, y_max = Y_LIMITS[axis_name]
        pi.setYRange(y_min, y_max, padding=0.0)
        legend = pi.addLegend()
        legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))
        return plot

    def _fmt(self, value: float) -> str:
        if not math.isfinite(float(value)):
            return "--"
        return f"{float(value):.4f}"

    def _set_curve_data(
        self,
        curve: pg.PlotDataItem,
        t_values: np.ndarray,
        y_values: np.ndarray,
    ) -> None:
        finite_mask = np.isfinite(t_values) & np.isfinite(y_values)
        if not np.any(finite_mask):
            curve.setData([], [])
            return
        curve.setData(t_values[finite_mask], y_values[finite_mask])

    def update_all(self) -> None:
        arrays = self.rosbuf.get_arrays()
        summary = self.rosbuf.get_summary()

        t = arrays["t"]
        if t.size == 0:
            return

        tmax = float(t[-1])
        window = X_HORIZON_SEC
        tmin = max(0.0, tmax - window)
        mask = t >= tmin
        t_win = t[mask]
        if t_win.size == 0:
            return

        for axis_name in AXES:
            for prefix in ("pure", "k_ep", "k_epi", "kalman", "adaptive", "u_neg", "ee"):
                key = f"{prefix}_{axis_name}"
                self._set_curve_data(self.curves[key], t_win, arrays[key][mask])
        self._set_curve_data(self.curves["desired_x"], t_win, arrays["desired_x"][mask])

        x_left = max(0.0, tmax - window)
        x_right = x_left + window
        for plot in self.plots:
            plot.setXRange(x_left, x_right, padding=0.0)

        self.info_label.setText(
            "Latest  "
            f"Fx: des={self._fmt(summary['desired_x'])}, pure={self._fmt(summary['pure_x'])}, "
            f"k_ep={self._fmt(summary['k_ep_x'])}, k_epi={self._fmt(summary['k_epi_x'])}, "
            f"kalman={self._fmt(summary['kalman_x'])}, adaptive={self._fmt(summary['adaptive_x'])}, "
            f"ee={self._fmt(summary['ee_x'])}, u={self._fmt(summary['u_neg_x'])}   "
            f"Fy: pure={self._fmt(summary['pure_y'])}, k_ep={self._fmt(summary['k_ep_y'])}, "
            f"k_epi={self._fmt(summary['k_epi_y'])}, kalman={self._fmt(summary['kalman_y'])}, "
            f"adaptive={self._fmt(summary['adaptive_y'])}, ee={self._fmt(summary['ee_y'])}, "
            f"u={self._fmt(summary['u_neg_y'])}   "
            f"Fz: pure={self._fmt(summary['pure_z'])}, k_ep={self._fmt(summary['k_ep_z'])}, "
            f"k_epi={self._fmt(summary['k_epi_z'])}, kalman={self._fmt(summary['kalman_z'])}, "
            f"adaptive={self._fmt(summary['adaptive_z'])}, ee={self._fmt(summary['ee_z'])}, "
            f"u={self._fmt(summary['u_neg_z'])}"
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
