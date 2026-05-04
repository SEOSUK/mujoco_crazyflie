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
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from std_msgs.msg import Float32, Float32MultiArray


def quat_xyzw_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
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


def wrench_torque(msg: WrenchStamped) -> np.ndarray:
    return np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z], dtype=float)


def finite_norm(v: np.ndarray) -> float:
    if not np.all(np.isfinite(v)):
        return math.nan
    return float(np.linalg.norm(v))


def nan_vec() -> np.ndarray:
    return np.full(3, math.nan, dtype=float)


class ROSDataBuffer(Node):
    def __init__(self):
        super().__init__("normal_vector_mob")

        self.declare_parameter("history_sec", 3.0)
        self.declare_parameter("update_hz", 30.0)
        self.declare_parameter("render_hz", 3.0)
        self.declare_parameter("pose_topic", "/crazyflie/out/pose")
        self.declare_parameter("input_topic", "/crazyflie/in/input")
        self.declare_parameter("vel_cmd_topic", "/su/debug/contact_vel_cmd")
        self.declare_parameter("vel_actual_topic", "/su/debug/contact_vel_actual")
        self.declare_parameter("force_cmd_topic", "su/cmd_force")
        self.declare_parameter("force_actual_topic", "/su/contact_force_x")
        self.declare_parameter("pure_mob_topic", "/crazyflie/out/mob_2nd_tau_base")
        self.declare_parameter("consistency_topic", "/crazyflie/out/mob_2nd_tau")
        self.declare_parameter("consistency_match_topic", "/crazyflie/out/mob_2nd_tau_consistency")
        self.declare_parameter("end_effector_offset", [0.1, 0.0, 0.04])

        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.render_hz = max(0.1, float(self.get_parameter("render_hz").value))

        ee_offset = list(self.get_parameter("end_effector_offset").value)
        if len(ee_offset) != 3:
            ee_offset = [0.1, 0.0, 0.04]
        self.ee_offset_body = np.array(ee_offset, dtype=float)

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(80, int(self.history_sec * self.update_hz) + 4)

        keys = [
            "c_hat_vy_cmd", "c_hat_vy_act",
            "c_hat_vz_cmd", "c_hat_vz_act",
            "c_hat_fx_cmd", "c_hat_fx_act",
            "ctrl_tau_x", "ctrl_tau_y", "ctrl_tau_z", "ctrl_tau_norm",
            "tauhat_x", "tauhat_y", "tauhat_z", "tauhat_norm",
            "rxf_pure_x", "rxf_pure_y", "rxf_pure_z",
            "rxf_con_x", "rxf_con_y", "rxf_con_z",
            "rxf_pure_norm", "rxf_con_norm",
            "eps_pure_x", "eps_pure_y", "eps_pure_z",
            "eps_con_x", "eps_con_y", "eps_con_z",
            "eps_pure_norm", "eps_con_norm",
        ]
        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for key in keys:
            self.data[key] = deque(maxlen=self.maxlen)

        self.latest = {key: math.nan for key in keys}

        self.rot_body_to_world = np.eye(3, dtype=float)
        self.r_world = self.ee_offset_body.copy()
        self.pure_force = nan_vec()
        self.consistency_force = nan_vec()
        self.tauhat_world = nan_vec()
        self.rxf_consistency = nan_vec()

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
            Vector3Stamped,
            str(self.get_parameter("vel_cmd_topic").value),
            self.cb_contact_vel_cmd,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("vel_actual_topic").value),
            self.cb_contact_vel_actual,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("force_cmd_topic").value),
            self.cb_force_cmd,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("force_actual_topic").value),
            self.cb_force_actual,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("pure_mob_topic").value),
            self.cb_pure_mob,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("consistency_topic").value),
            self.cb_consistency,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("consistency_match_topic").value),
            self.cb_match,
            10,
        )

        self.timer = self.create_timer(1.0 / self.update_hz, self.log_snapshot)
        self.get_logger().info("normal_vector_mob started")

    def cb_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        rot = quat_xyzw_to_rotmat(q.x, q.y, q.z, q.w)
        with self.lock:
            self.rot_body_to_world = rot
            self.r_world = rot @ self.ee_offset_body

    def cb_input(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        tau_body = np.array(msg.data[:3], dtype=float)
        with self.lock:
            tau_world = -(self.rot_body_to_world @ tau_body)
            self.latest["ctrl_tau_x"] = float(tau_world[0])
            self.latest["ctrl_tau_y"] = float(tau_world[1])
            self.latest["ctrl_tau_z"] = float(tau_world[2])
            self.latest["ctrl_tau_norm"] = finite_norm(tau_world)

    def cb_contact_vel_cmd(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["c_hat_vy_cmd"] = float(msg.vector.y)
            self.latest["c_hat_vz_cmd"] = float(msg.vector.z)

    def cb_contact_vel_actual(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["c_hat_vy_act"] = float(msg.vector.y)
            self.latest["c_hat_vz_act"] = float(msg.vector.z)

    def cb_force_cmd(self, msg: Float32) -> None:
        with self.lock:
            self.latest["c_hat_fx_cmd"] = float(msg.data)

    def cb_force_actual(self, msg: Float32) -> None:
        with self.lock:
            self.latest["c_hat_fx_act"] = float(msg.data)

    def cb_pure_mob(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.pure_force = wrench_force(msg)

    def cb_consistency(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.consistency_force = wrench_force(msg)

    def cb_match(self, msg: WrenchStamped) -> None:
        tauhat = wrench_force(msg)
        rxf_con = wrench_torque(msg)
        if not np.all(np.isfinite(tauhat)) or not np.all(np.isfinite(rxf_con)):
            return
        with self.lock:
            self.tauhat_world = tauhat
            self.rxf_consistency = rxf_con

    def log_snapshot(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        with self.lock:
            tauhat = self.tauhat_world.copy()
            r_world = self.r_world.copy()
            pure_force = self.pure_force.copy()
            consistency_force = self.consistency_force.copy()
            rxf_con = self.rxf_consistency.copy()

            rxf_pure = np.cross(r_world, pure_force) if np.all(np.isfinite(r_world)) and np.all(np.isfinite(pure_force)) else nan_vec()
            rxf_consistency_from_force = (
                np.cross(r_world, consistency_force)
                if np.all(np.isfinite(r_world)) and np.all(np.isfinite(consistency_force))
                else nan_vec()
            )
            eps_pure = tauhat - rxf_pure if np.all(np.isfinite(tauhat)) and np.all(np.isfinite(rxf_pure)) else nan_vec()
            eps_con = (
                tauhat - rxf_consistency_from_force
                if np.all(np.isfinite(tauhat)) and np.all(np.isfinite(rxf_consistency_from_force))
                else nan_vec()
            )

            self.latest["tauhat_x"] = float(tauhat[0])
            self.latest["tauhat_y"] = float(tauhat[1])
            self.latest["tauhat_z"] = float(tauhat[2])
            self.latest["tauhat_norm"] = finite_norm(tauhat)
            self.latest["rxf_pure_norm"] = finite_norm(rxf_pure)
            self.latest["rxf_pure_x"] = float(rxf_pure[0])
            self.latest["rxf_pure_y"] = float(rxf_pure[1])
            self.latest["rxf_pure_z"] = float(rxf_pure[2])
            self.latest["rxf_con_norm"] = finite_norm(rxf_con)
            self.latest["rxf_con_x"] = float(rxf_con[0])
            self.latest["rxf_con_y"] = float(rxf_con[1])
            self.latest["rxf_con_z"] = float(rxf_con[2])
            self.latest["eps_pure_norm"] = finite_norm(eps_pure)
            self.latest["eps_pure_x"] = float(eps_pure[0])
            self.latest["eps_pure_y"] = float(eps_pure[1])
            self.latest["eps_pure_z"] = float(eps_pure[2])
            self.latest["eps_con_norm"] = finite_norm(eps_con)
            self.latest["eps_con_x"] = float(eps_con[0])
            self.latest["eps_con_y"] = float(eps_con[1])
            self.latest["eps_con_z"] = float(eps_con[2])

            self.data["t"].append(t)
            for key, value in self.latest.items():
                self.data[key].append(float(value))

    def get_arrays(self) -> Dict[str, np.ndarray]:
        with self.lock:
            return {k: np.array(v, dtype=float) for k, v in self.data.items()}

    def get_summary(self) -> Dict[str, float]:
        with self.lock:
            return dict(self.latest)


class PlotWindow(QMainWindow):
    def __init__(self, rosbuf: ROSDataBuffer):
        super().__init__()
        self.rosbuf = rosbuf
        self.setWindowTitle("normal_vector_mob")

        pg.setConfigOptions(antialias=True)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        self.info_label = QLabel("Waiting for MOB panel data...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.info_label.setWordWrap(True)
        self.info_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        root.addWidget(self.info_label)

        root.addWidget(self._build_control_panel(), stretch=0)
        root.addWidget(self._build_torque_components_panel(), stretch=0)
        root.addWidget(self._build_torque_match_panel(), stretch=0)
        root.addWidget(self._build_epsilon_panel(), stretch=1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(max(1, int(round(1000.0 / self.rosbuf.render_hz))))

        self.setFixedWidth(720)
        self.resize(720, 1460)

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

    def _make_plot(self, title: str, ylabel: str, min_height: int = 120):
        plot = pg.PlotWidget()
        plot.setBackground("w")
        plot.setMinimumWidth(1)
        plot.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        plot.setMinimumHeight(min_height)
        pi = plot.getPlotItem()
        pi.setTitle(title)
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("left", ylabel)
        pi.setLabel("bottom", "time [s]")
        pi.getAxis("left").setPen(pg.mkPen("k"))
        pi.getAxis("bottom").setPen(pg.mkPen("k"))
        pi.getAxis("left").setTextPen(pg.mkPen("k"))
        pi.getAxis("bottom").setTextPen(pg.mkPen("k"))
        legend = pi.addLegend()
        legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))
        return plot

    def _build_control_panel(self) -> QWidget:
        frame = self._make_group_frame("Control Status")
        outer = frame.layout()

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.plot_vy = self._make_plot("c_hat_vy", "vel [m/s]", min_height=120)
        self.curve_vy_cmd = self.plot_vy.plot(name="desired", pen=pg.mkPen((40, 90, 220), width=2))
        self.curve_vy_act = self.plot_vy.plot(name="actual", pen=pg.mkPen((220, 50, 50), width=2))
        self.plot_vy.setYRange(-0.3, 0.3, padding=0.0)

        self.plot_vz = self._make_plot("c_hat_vz", "vel [m/s]", min_height=120)
        self.curve_vz_cmd = self.plot_vz.plot(name="desired", pen=pg.mkPen((50, 160, 70), width=2))
        self.curve_vz_act = self.plot_vz.plot(name="actual", pen=pg.mkPen((220, 50, 50), width=2))
        self.plot_vz.setYRange(-0.3, 0.3, padding=0.0)

        self.plot_fx = self._make_plot("c_hat_fx", "force [N]", min_height=120)
        self.curve_fx_cmd = self.plot_fx.plot(name="desired", pen=pg.mkPen((40, 90, 220), width=2))
        self.curve_fx_act = self.plot_fx.plot(name="actual", pen=pg.mkPen((220, 50, 50), width=2))
        self.plot_fx.setYRange(-0.1, 0.1, padding=0.0)

        grid.addWidget(self.plot_vy, 0, 0)
        grid.addWidget(self.plot_vz, 0, 1)
        grid.addWidget(self.plot_fx, 0, 2)
        outer.addLayout(grid)
        return frame

    def _build_torque_components_panel(self) -> QWidget:
        frame = self._make_group_frame("Torque Components")
        outer = frame.layout()

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.torque_component_plots = []
        self.torque_component_curves = {}
        axis_colors = {
            "input": pg.mkPen((40, 90, 220), width=2),
            "mob": pg.mkPen((220, 50, 50), width=2),
        }

        for col, axis in enumerate(("x", "y", "z")):
            plot = self._make_plot(f"torque {axis}", "torque [N m]", min_height=140)
            self.torque_component_plots.append(plot)
            self.torque_component_curves[f"ctrl_tau_{axis}"] = plot.plot(
                name="actuator input", pen=axis_colors["input"])
            self.torque_component_curves[f"tauhat_{axis}"] = plot.plot(
                name="MOB estimate", pen=axis_colors["mob"])
            plot.setYRange(-0.02, 0.02, padding=0.0)
            grid.addWidget(plot, 0, col)

        outer.addLayout(grid)
        return frame

    def _build_torque_match_panel(self) -> QWidget:
        frame = self._make_group_frame("Torque vs r x F")
        outer = frame.layout()

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.torque_match_plots = []
        self.torque_match_curves = {}
        colors = {
            "mob": pg.mkPen((120, 80, 220), width=2),
            "pure": pg.mkPen((40, 150, 75), width=2),
            "con": pg.mkPen((220, 70, 45), width=2),
        }

        for col, axis in enumerate(("x", "y", "z")):
            plot = self._make_plot(f"torque match {axis}", "torque [N m]", min_height=180)
            self.torque_match_plots.append(plot)
            self.torque_match_curves[f"tauhat_{axis}"] = plot.plot(
                name="MOB estimate", pen=colors["mob"])
            self.torque_match_curves[f"rxf_pure_{axis}"] = plot.plot(
                name="r x F (pure MOB)", pen=colors["pure"])
            self.torque_match_curves[f"rxf_con_{axis}"] = plot.plot(
                name="r x F (consistency)", pen=colors["con"])
            plot.setYRange(-0.02, 0.02, padding=0.0)
            grid.addWidget(plot, 0, col)

        outer.addLayout(grid)
        return frame

    def _build_epsilon_panel(self) -> QWidget:
        frame = self._make_group_frame("Epsilon Comparison")
        outer = frame.layout()

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.epsilon_plots = []
        self.epsilon_curves = {}
        colors = {
            "pure": pg.mkPen((40, 90, 220), width=2),
            "con": pg.mkPen((220, 70, 45), width=2),
        }

        for col, axis in enumerate(("x", "y", "z")):
            plot = self._make_plot(f"epsilon {axis}", "torque [N m]", min_height=210)
            self.epsilon_plots.append(plot)
            self.epsilon_curves[f"eps_pure_{axis}"] = plot.plot(
                name="pure MOB", pen=colors["pure"])
            self.epsilon_curves[f"eps_con_{axis}"] = plot.plot(
                name="consistency", pen=colors["con"])
            plot.setYRange(-0.02, 0.02, padding=0.0)
            grid.addWidget(plot, 0, col)

        outer.addLayout(grid)
        return frame

    def _fmt(self, value: float, precision: int = 4) -> str:
        if not math.isfinite(float(value)):
            return "--"
        return f"{float(value):.{precision}f}"

    def update_all(self) -> None:
        arr = self.rosbuf.get_arrays()
        latest = self.rosbuf.get_summary()

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

        self.curve_vy_cmd.setData(t_win, arr["c_hat_vy_cmd"][mask])
        self.curve_vy_act.setData(t_win, arr["c_hat_vy_act"][mask])
        self.curve_vz_cmd.setData(t_win, arr["c_hat_vz_cmd"][mask])
        self.curve_vz_act.setData(t_win, arr["c_hat_vz_act"][mask])
        self.curve_fx_cmd.setData(t_win, arr["c_hat_fx_cmd"][mask])
        self.curve_fx_act.setData(t_win, arr["c_hat_fx_act"][mask])

        for axis in ("x", "y", "z"):
            self.torque_component_curves[f"ctrl_tau_{axis}"].setData(t_win, arr[f"ctrl_tau_{axis}"][mask])
            self.torque_component_curves[f"tauhat_{axis}"].setData(t_win, arr[f"tauhat_{axis}"][mask])
            self.torque_match_curves[f"tauhat_{axis}"].setData(t_win, arr[f"tauhat_{axis}"][mask])
            self.torque_match_curves[f"rxf_pure_{axis}"].setData(t_win, arr[f"rxf_pure_{axis}"][mask])
            self.torque_match_curves[f"rxf_con_{axis}"].setData(t_win, arr[f"rxf_con_{axis}"][mask])
            self.epsilon_curves[f"eps_pure_{axis}"].setData(t_win, arr[f"eps_pure_{axis}"][mask])
            self.epsilon_curves[f"eps_con_{axis}"].setData(t_win, arr[f"eps_con_{axis}"][mask])

        x_left = max(0.0, tmax - window)
        x_right = tmax if tmax >= window else window
        for plot in [
            self.plot_vy, self.plot_vz, self.plot_fx,
            *self.torque_component_plots,
            *self.torque_match_plots,
            *self.epsilon_plots,
        ]:
            plot.setXRange(x_left, x_right, padding=0.0)

        self.info_label.setText(
            f"vy(cmd/act)={self._fmt(latest['c_hat_vy_cmd'])}/{self._fmt(latest['c_hat_vy_act'])}   "
            f"vz(cmd/act)={self._fmt(latest['c_hat_vz_cmd'])}/{self._fmt(latest['c_hat_vz_act'])}   "
            f"fx(cmd/act)={self._fmt(latest['c_hat_fx_cmd'])}/{self._fmt(latest['c_hat_fx_act'])}   "
            f"|tau_in|={self._fmt(latest['ctrl_tau_norm'])}   "
            f"|tau_mob|={self._fmt(latest['tauhat_norm'])}   "
            f"|rXF_pure|={self._fmt(latest['rxf_pure_norm'])}   "
            f"|rXF_con|={self._fmt(latest['rxf_con_norm'])}   "
            f"|eps_pure|={self._fmt(latest['eps_pure_norm'])}   "
            f"|eps_con|={self._fmt(latest['eps_con_norm'])}"
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
