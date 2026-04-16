#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import signal
import threading
import argparse
from collections import deque
from typing import Dict, List, Tuple

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
    QGridLayout,
)

import pyqtgraph as pg

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray


def quat_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ROSDataBuffer(Node):
    def __init__(self):
        super().__init__("data_logging_drone_states")

        self.declare_parameter("history_sec", 5.0)
        self.declare_parameter("update_hz", 30.0)
        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(100, int(self.history_sec * self.update_hz) + 20)

        keys = [
            "x_des", "y_des", "z_des",
            "x_act", "y_act", "z_act",

            "vx_des", "vy_des", "vz_des",
            "vx_act", "vy_act", "vz_act",

            "ax_act", "ay_act", "az_act",

            "roll_des", "pitch_des", "yaw_des",
            "roll_act", "pitch_act", "yaw_act",

            "wx_des", "wy_des", "wz_des",
            "wx_act", "wy_act", "wz_act",

            "alphax_act", "alphay_act", "alphaz_act",
        ]

        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for k in keys:
            self.data[k] = deque(maxlen=self.maxlen)

        self.latest = {k: np.nan for k in keys}

        # desired
        self.create_subscription(Float64MultiArray, "/crazyflie/in/pos_cmd", self.cb_pos_cmd, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/debug/rpy_des", self.cb_rpy_des, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/debug/vel_des", self.cb_vel_des, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/debug/ang_vel_des", self.cb_ang_vel_des, 10)

        # actual
        self.create_subscription(PoseStamped, "/crazyflie/out/pose", self.cb_pose, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/out/vel", self.cb_vel, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/out/acc", self.cb_acc, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/out/ang_vel", self.cb_ang_vel, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/out/ang_acc", self.cb_ang_acc, 10)

        self.timer = self.create_timer(1.0 / self.update_hz, self.log_snapshot)
        self.get_logger().info("ROSDataBuffer started")

    def cb_pos_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 4:
            with self.lock:
                self.latest["x_des"] = float(msg.data[0])
                self.latest["y_des"] = float(msg.data[1])
                self.latest["z_des"] = float(msg.data[2])

    def cb_rpy_des(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["roll_des"] = float(msg.vector.x)
            self.latest["pitch_des"] = float(msg.vector.y)
            self.latest["yaw_des"] = float(msg.vector.z)

    def cb_vel_des(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["vx_des"] = float(msg.vector.x)
            self.latest["vy_des"] = float(msg.vector.y)
            self.latest["vz_des"] = float(msg.vector.z)

    def cb_ang_vel_des(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["wx_des"] = float(msg.vector.x)
            self.latest["wy_des"] = float(msg.vector.y)
            self.latest["wz_des"] = float(msg.vector.z)

    def cb_pose(self, msg: PoseStamped) -> None:
        roll, pitch, yaw = quat_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        with self.lock:
            self.latest["x_act"] = float(msg.pose.position.x)
            self.latest["y_act"] = float(msg.pose.position.y)
            self.latest["z_act"] = float(msg.pose.position.z)
            self.latest["roll_act"] = roll
            self.latest["pitch_act"] = pitch
            self.latest["yaw_act"] = yaw

    def cb_vel(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["vx_act"] = float(msg.vector.x)
            self.latest["vy_act"] = float(msg.vector.y)
            self.latest["vz_act"] = float(msg.vector.z)

    def cb_acc(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["ax_act"] = float(msg.vector.x)
            self.latest["ay_act"] = float(msg.vector.y)
            self.latest["az_act"] = float(msg.vector.z)

    def cb_ang_vel(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["wx_act"] = float(msg.vector.x)
            self.latest["wy_act"] = float(msg.vector.y)
            self.latest["wz_act"] = float(msg.vector.z)

    def cb_ang_acc(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["alphax_act"] = float(msg.vector.x)
            self.latest["alphay_act"] = float(msg.vector.y)
            self.latest["alphaz_act"] = float(msg.vector.z)

    def log_snapshot(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        with self.lock:
            self.data["t"].append(t)
            for key in self.latest:
                self.data[key].append(self.latest[key])

    def get_arrays(self) -> Dict[str, np.ndarray]:
        with self.lock:
            return {k: np.array(v, dtype=float) for k, v in self.data.items()}


class PlotWindow(QMainWindow):
    def __init__(self, rosbuf: ROSDataBuffer):
        super().__init__()
        self.rosbuf = rosbuf
        self.setWindowTitle("data_logging_drone_states")

        pg.setConfigOptions(antialias=True)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        self.info_label = QLabel("Waiting for data...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        root.addWidget(self.info_label)

        # 전체 그룹 배치: 2행 3열
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)
        root.addLayout(grid)

        self.plots: Dict[str, Tuple[pg.PlotWidget, List[pg.PlotDataItem]]] = {}

        lw = 2.0
        pen_des = pg.mkPen(color=(40, 90, 220), width=lw)
        pen_act = pg.mkPen(color=(220, 50, 50), width=lw)
        pen_x = pg.mkPen(color=(220, 50, 50), width=lw)
        pen_y = pg.mkPen(color=(50, 160, 70), width=lw)
        pen_z = pg.mkPen(color=(40, 90, 220), width=lw)

        def make_plot(y_label: str, curve_specs, y_range=None):
            w = pg.PlotWidget()
            w.setBackground("w")
            w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            pi = w.getPlotItem()
            pi.showGrid(x=True, y=True, alpha=0.25)
            pi.setLabel("bottom", "time [s]")
            pi.setLabel("left", y_label)
            pi.getAxis("left").setPen(pg.mkPen("k"))
            pi.getAxis("bottom").setPen(pg.mkPen("k"))
            pi.getAxis("left").setTextPen(pg.mkPen("k"))
            pi.getAxis("bottom").setTextPen(pg.mkPen("k"))

            if y_range is not None:
                pi.enableAutoRange(axis="y", enable=False)
                pi.setYRange(float(y_range[0]), float(y_range[1]), padding=0.0)

            legend = pi.addLegend()
            legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))

            curves = []
            for name, pen in curve_specs:
                curves.append(w.plot(name=name, pen=pen))
            return w, curves

        # position
        self.plots["pos_x"] = make_plot("[m]", [("x_des", pen_des), ("x_act", pen_act)], (-1.0, 1.0))
        self.plots["pos_y"] = make_plot("[m]", [("y_des", pen_des), ("y_act", pen_act)], (-1.0, 1.0))
        self.plots["pos_z"] = make_plot("[m]", [("z_des", pen_des), ("z_act", pen_act)], (-1.0, 1.0))

        # velocity
        self.plots["vel_x"] = make_plot("[m/s]", [("vx_des", pen_des), ("vx_act", pen_act)], (-1.0, 1.0))
        self.plots["vel_y"] = make_plot("[m/s]", [("vy_des", pen_des), ("vy_act", pen_act)], (-1.0, 1.0))
        self.plots["vel_z"] = make_plot("[m/s]", [("vz_des", pen_des), ("vz_act", pen_act)], (-1.0, 1.0))

        # acceleration
        self.plots["acc_x"] = make_plot("[m/s²]", [("ax_act", pen_x)], (-5.0, 5.0))
        self.plots["acc_y"] = make_plot("[m/s²]", [("ay_act", pen_y)], (-5.0, 5.0))
        self.plots["acc_z"] = make_plot("[m/s²]", [("az_act", pen_z)], (-5.0, 5.0))

        # attitude
        self.plots["att_r"] = make_plot("[rad]", [("roll_des", pen_des), ("roll_act", pen_act)], (-0.5, 0.5))
        self.plots["att_p"] = make_plot("[rad]", [("pitch_des", pen_des), ("pitch_act", pen_act)], (-0.5, 0.5))
        self.plots["att_y"] = make_plot("[rad]", [("yaw_des", pen_des), ("yaw_act", pen_act)], (-0.5, 0.5))

        # angular velocity
        self.plots["w_x"] = make_plot("[rad/s]", [("wx_des", pen_des), ("wx_act", pen_act)], (-5.0, 5.0))
        self.plots["w_y"] = make_plot("[rad/s]", [("wy_des", pen_des), ("wy_act", pen_act)], (-5.0, 5.0))
        self.plots["w_z"] = make_plot("[rad/s]", [("wz_des", pen_des), ("wz_act", pen_act)], (-5.0, 5.0))

        # angular acceleration
        self.plots["alpha_x"] = make_plot("[rad/s²]", [("alphax_act", pen_x)], (-20.0, 20.0))
        self.plots["alpha_y"] = make_plot("[rad/s²]", [("alphay_act", pen_y)], (-20.0, 20.0))
        self.plots["alpha_z"] = make_plot("[rad/s²]", [("alphaz_act", pen_z)], (-20.0, 20.0))


        def make_panel_vertical(panel_title: str, keys: List[str]) -> QWidget:
            outer = QWidget()
            outer_layout = QVBoxLayout()
            outer_layout.setContentsMargins(0, 0, 0, 0)
            outer_layout.setSpacing(6)
            outer.setLayout(outer_layout)

            title_label = QLabel(panel_title)
            title_label.setAlignment(Qt.AlignCenter)
            title_label.setStyleSheet("""
                QLabel {
                    font-size: 16px;
                    font-weight: bold;
                    padding: 4px;
                }
            """)
            outer_layout.addWidget(title_label)

            inner = QWidget()
            g = QGridLayout()
            g.setContentsMargins(0, 0, 0, 0)
            g.setHorizontalSpacing(4)
            g.setVerticalSpacing(6)
            inner.setLayout(g)

            for i, k in enumerate(keys):
                plot_widget, _ = self.plots[k]
                g.addWidget(plot_widget, i, 0)

                bottom_axis = plot_widget.getPlotItem().getAxis("bottom")
                if i < len(keys) - 1:
                    bottom_axis.setStyle(showValues=False)
                    plot_widget.getPlotItem().setLabel("bottom", "")
                else:
                    bottom_axis.setStyle(showValues=True)
                    plot_widget.getPlotItem().setLabel("bottom", "time [s]")

            outer_layout.addWidget(inner)
            return outer

        panel_pos = make_panel_vertical("Drone Position", ["pos_x", "pos_y", "pos_z"])
        panel_vel = make_panel_vertical("Drone Velocity", ["vel_x", "vel_y", "vel_z"])
        panel_acc = make_panel_vertical("Drone Acceleration", ["acc_x", "acc_y", "acc_z"])
        panel_att = make_panel_vertical("Drone Attitude", ["att_r", "att_p", "att_y"])
        panel_w = make_panel_vertical("Drone Angular Velocity", ["w_x", "w_y", "w_z"])
        panel_alpha = make_panel_vertical("Drone Angular Acceleration", ["alpha_x", "alpha_y", "alpha_z"])

        grid.addWidget(panel_pos, 0, 0)
        grid.addWidget(panel_vel, 0, 1)
        grid.addWidget(panel_acc, 0, 2)
        grid.addWidget(panel_att, 1, 0)
        grid.addWidget(panel_w, 1, 1)
        grid.addWidget(panel_alpha, 1, 2)

        # 전체 x축 링크
        base_plot = self.plots["pos_x"][0]
        for key, (plot_widget, _) in self.plots.items():
            if key != "pos_x":
                plot_widget.setXLink(base_plot)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)

        self.resize(1800, 1400)

    def _set_plot(self, key: str, x: np.ndarray, ys: List[np.ndarray]) -> None:
        _, curves = self.plots[key]
        for curve, y in zip(curves, ys):
            curve.setData(x, y)

    def update_plots(self) -> None:
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

        self._set_plot("pos_x", t_win, [arr["x_des"][mask], arr["x_act"][mask]])
        self._set_plot("pos_y", t_win, [arr["y_des"][mask], arr["y_act"][mask]])
        self._set_plot("pos_z", t_win, [arr["z_des"][mask], arr["z_act"][mask]])

        self._set_plot("vel_x", t_win, [arr["vx_des"][mask], arr["vx_act"][mask]])
        self._set_plot("vel_y", t_win, [arr["vy_des"][mask], arr["vy_act"][mask]])
        self._set_plot("vel_z", t_win, [arr["vz_des"][mask], arr["vz_act"][mask]])

        self._set_plot("acc_x", t_win, [arr["ax_act"][mask]])
        self._set_plot("acc_y", t_win, [arr["ay_act"][mask]])
        self._set_plot("acc_z", t_win, [arr["az_act"][mask]])

        self._set_plot("att_r", t_win, [arr["roll_des"][mask], arr["roll_act"][mask]])
        self._set_plot("att_p", t_win, [arr["pitch_des"][mask], arr["pitch_act"][mask]])
        self._set_plot("att_y", t_win, [arr["yaw_des"][mask], arr["yaw_act"][mask]])

        self._set_plot("w_x", t_win, [arr["wx_des"][mask], arr["wx_act"][mask]])
        self._set_plot("w_y", t_win, [arr["wy_des"][mask], arr["wy_act"][mask]])
        self._set_plot("w_z", t_win, [arr["wz_des"][mask], arr["wz_act"][mask]])

        self._set_plot("alpha_x", t_win, [arr["alphax_act"][mask]])
        self._set_plot("alpha_y", t_win, [arr["alphay_act"][mask]])
        self._set_plot("alpha_z", t_win, [arr["alphaz_act"][mask]])

        x_left = tmax - window
        x_right = tmax
        if x_left < 0.0:
            x_left = 0.0
            x_right = window

        for plot_widget, _ in self.plots.values():
            plot_widget.setXRange(x_left, x_right, padding=0.0)

        self.info_label.setText(
            f"t={t[-1]:.2f}s   "
            f"pos=({arr['x_act'][-1]:.3f}, {arr['y_act'][-1]:.3f}, {arr['z_act'][-1]:.3f})   "
            f"vel=({arr['vx_act'][-1]:.3f}, {arr['vy_act'][-1]:.3f}, {arr['vz_act'][-1]:.3f})   "
            f"att=({arr['roll_act'][-1]:.3f}, {arr['pitch_act'][-1]:.3f}, {arr['yaw_act'][-1]:.3f})   "
            f"w=({arr['wx_act'][-1]:.3f}, {arr['wy_act'][-1]:.3f}, {arr['wz_act'][-1]:.3f})"
        )


def main() -> int:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--window-x", type=int, default=0)
    parser.add_argument("--window-y", type=int, default=40)
    parser.add_argument("--window-width", type=int, default=880)
    parser.add_argument("--window-height", type=int, default=980)
    args, ros_args = parser.parse_known_args()

    app = QApplication.instance()
    if app is None:
        app = QApplication([sys.argv[0]])

    app.setStyleSheet("""
    QWidget { background: #ffffff; color: #111111; }
    QMainWindow { background: #ffffff; }
    QLabel {
        font-family: monospace;
        font-size: 12px;
        padding: 2px 4px 2px 4px;
    }
    """)

    rclpy.init(args=ros_args)
    rosbuf = ROSDataBuffer()

    def _spin_ros():
        try:
            rclpy.spin(rosbuf)
        except ExternalShutdownException:
            pass

    ros_thread = threading.Thread(target=_spin_ros, daemon=True)
    ros_thread.start()

    win = PlotWindow(rosbuf)
    win.resize(args.window_width, args.window_height)
    win.move(args.window_x, args.window_y)
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
