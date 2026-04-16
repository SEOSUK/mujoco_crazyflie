#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import signal
import threading
from collections import deque
from typing import Dict, List, Tuple

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
    QGridLayout,
)

import pyqtgraph as pg

from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray


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
        super().__init__("realtime_plotter_pyqtgraph")

        self.declare_parameter("history_sec", 5.0)
        self.declare_parameter("update_hz", 30.0)
        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(100, int(self.history_sec * self.update_hz) + 20)

        self.data: Dict[str, deque] = {
            "t": deque(maxlen=self.maxlen),

            "x_des": deque(maxlen=self.maxlen),
            "y_des": deque(maxlen=self.maxlen),
            "z_des": deque(maxlen=self.maxlen),

            "x_act": deque(maxlen=self.maxlen),
            "y_act": deque(maxlen=self.maxlen),
            "z_act": deque(maxlen=self.maxlen),

            "roll_des": deque(maxlen=self.maxlen),
            "pitch_des": deque(maxlen=self.maxlen),
            "yaw_des": deque(maxlen=self.maxlen),

            "roll_act": deque(maxlen=self.maxlen),
            "pitch_act": deque(maxlen=self.maxlen),
            "yaw_act": deque(maxlen=self.maxlen),

            "force_cmd": deque(maxlen=self.maxlen),
            "force_contact_x": deque(maxlen=self.maxlen),

            "force_wx": deque(maxlen=self.maxlen),
            "force_wy": deque(maxlen=self.maxlen),
            "force_wz": deque(maxlen=self.maxlen),

            "m1": deque(maxlen=self.maxlen),
            "m2": deque(maxlen=self.maxlen),
            "m3": deque(maxlen=self.maxlen),
            "m4": deque(maxlen=self.maxlen),
        }

        self.latest = {
            "x_des": np.nan, "y_des": np.nan, "z_des": np.nan,
            "x_act": np.nan, "y_act": np.nan, "z_act": np.nan,

            "roll_des": np.nan, "pitch_des": np.nan, "yaw_des": np.nan,
            "roll_act": np.nan, "pitch_act": np.nan, "yaw_act": np.nan,

            "force_cmd": np.nan,
            "force_contact_x": np.nan,

            "force_wx": np.nan,
            "force_wy": np.nan,
            "force_wz": np.nan,

            "m1": np.nan, "m2": np.nan, "m3": np.nan, "m4": np.nan,
        }

        self.create_subscription(Float64MultiArray, "/crazyflie/in/pos_cmd", self.cb_pos_cmd, 10)
        self.create_subscription(Vector3Stamped, "/crazyflie/debug/rpy_des", self.cb_rpy_des, 10)
        self.create_subscription(Float32, "su/cmd_force", self.cb_cmd_force, 10)
        self.create_subscription(Float32, "su/contact_force_x", self.cb_contact_force_x, 10)
        self.create_subscription(PoseStamped, "/crazyflie/out/pose", self.cb_pose, 10)
        self.create_subscription(WrenchStamped, "/crazyflie/out/EE_contact_force_filt", self.cb_contact_force_world, 10)
        self.create_subscription(Float32MultiArray, "/crazyflie/out/motor_thrust", self.cb_motor_thrust, 10)

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

    def cb_cmd_force(self, msg: Float32) -> None:
        with self.lock:
            self.latest["force_cmd"] = float(msg.data)

    def cb_contact_force_x(self, msg: Float32) -> None:
        with self.lock:
            self.latest["force_contact_x"] = float(msg.data)

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

    def cb_contact_force_world(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.latest["force_wx"] = float(msg.wrench.force.x)
            self.latest["force_wy"] = float(msg.wrench.force.y)
            self.latest["force_wz"] = float(msg.wrench.force.z)

    def cb_motor_thrust(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            with self.lock:
                self.latest["m1"] = float(msg.data[0])
                self.latest["m2"] = float(msg.data[1])
                self.latest["m3"] = float(msg.data[2])
                self.latest["m4"] = float(msg.data[3])

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
        self.setWindowTitle("Crazyflie Real-Time Plotter")

        pg.setConfigOptions(antialias=True)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        self.info_label = QLabel("Waiting for data...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        root.addWidget(self.info_label)

        # 전체 2x2 사분면
        quad = QGridLayout()
        quad.setContentsMargins(0, 0, 0, 0)
        quad.setHorizontalSpacing(16)
        quad.setVerticalSpacing(16)
        root.addLayout(quad)

        # 각 사분면 내부 grid
        grid_lt = QGridLayout()  # left-top
        grid_lb = QGridLayout()  # left-bottom
        grid_rt = QGridLayout()  # right-top
        grid_rb = QGridLayout()  # right-bottom

        for g in (grid_lt, grid_lb, grid_rt, grid_rb):
            g.setContentsMargins(0, 0, 0, 0)
            g.setHorizontalSpacing(10)
            g.setVerticalSpacing(10)

        self.plots: Dict[str, Tuple[pg.PlotWidget, List[pg.PlotDataItem]]] = {}

        lw = 2.0
        pen_r = pg.mkPen(color=(220, 50, 50), width=lw)
        pen_b = pg.mkPen(color=(40, 90, 220), width=lw)
        pen_g = pg.mkPen(color=(50, 160, 70), width=lw)
        pen_p = pg.mkPen(color=(130, 70, 210), width=lw)

        def make_plot(title: str, y_label: str, curve_specs, y_range=None):
            w = pg.PlotWidget()
            w.setBackground("w")
            w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            pi = w.getPlotItem()
            pi.setTitle(title)
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

        # 왼쪽 위: position x y z
        self.plots["pos_x"] = make_plot("Position X", "[m]", [("x_des", pen_b), ("x_act", pen_r)], (-1.0, 1.0))
        self.plots["pos_y"] = make_plot("Position Y", "[m]", [("y_des", pen_b), ("y_act", pen_r)], (-1.0, 1.0))
        self.plots["pos_z"] = make_plot("Position Z", "[m]", [("z_des", pen_b), ("z_act", pen_r)], (-1.0, 1.0))

        # 왼쪽 아래: global force x y z + desired vs actual force
        self.plots["force_wx"] = make_plot("Global Force X", "[N]", [("force_wx", pen_r)], (-0.1, 0.1))
        self.plots["force_wy"] = make_plot("Global Force Y", "[N]", [("force_wy", pen_g)], (-0.1, 0.1))
        self.plots["force_wz"] = make_plot("Global Force Z", "[N]", [("force_wz", pen_b)], (-0.1, 0.1))
        self.plots["force_contact"] = make_plot(
            "Desired vs Actual Force",
            "[N]",
            [("force_cmd", pen_g), ("force_contact_x", pen_p)],
            (0.0, 0.1)
        )

        # 오른쪽 위: attitude 3개
        self.plots["att_roll"] = make_plot("Roll", "[rad]", [("roll_des", pen_b), ("roll_act", pen_r)], (-0.5, 0.5))
        self.plots["att_pitch"] = make_plot("Pitch", "[rad]", [("pitch_des", pen_b), ("pitch_act", pen_r)], (-0.5, 0.5))
        self.plots["att_yaw"] = make_plot("Yaw", "[rad]", [("yaw_des", pen_b), ("yaw_act", pen_r)], (-0.5, 0.5))

        # 오른쪽 아래: thrust 1 2 3 4
        self.plots["motor"] = make_plot(
            "Motor Thrust 1 2 3 4",
            "[N]",
            [("m1", pen_r), ("m2", pen_g), ("m3", pen_b), ("m4", pen_p)],
            (0.0, 0.2)
        )

        # 배치
        grid_lt.addWidget(self.plots["pos_x"][0], 0, 0)
        grid_lt.addWidget(self.plots["pos_y"][0], 0, 1)
        grid_lt.addWidget(self.plots["pos_z"][0], 0, 2)

        grid_lb.addWidget(self.plots["force_wx"][0], 0, 0)
        grid_lb.addWidget(self.plots["force_wy"][0], 0, 1)
        grid_lb.addWidget(self.plots["force_wz"][0], 0, 2)
        grid_lb.addWidget(self.plots["force_contact"][0], 1, 0, 1, 3)

        grid_rt.addWidget(self.plots["att_roll"][0], 0, 0)
        grid_rt.addWidget(self.plots["att_pitch"][0], 0, 1)
        grid_rt.addWidget(self.plots["att_yaw"][0], 0, 2)

        grid_rb.addWidget(self.plots["motor"][0], 0, 0)

        # 사분면 wrapper
        w_lt = QWidget(); w_lt.setLayout(grid_lt)
        w_lb = QWidget(); w_lb.setLayout(grid_lb)
        w_rt = QWidget(); w_rt.setLayout(grid_rt)
        w_rb = QWidget(); w_rb.setLayout(grid_rb)

        quad.addWidget(w_lt, 0, 0)
        quad.addWidget(w_rt, 0, 1)
        quad.addWidget(w_lb, 1, 0)
        quad.addWidget(w_rb, 1, 1)

        # x축 링크
        base_plot = self.plots["pos_x"][0]
        for key, (plot_widget, _) in self.plots.items():
            if key != "pos_x":
                plot_widget.setXLink(base_plot)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)

        self.resize(2200, 1100)

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

        self._set_plot("force_wx", t_win, [arr["force_wx"][mask]])
        self._set_plot("force_wy", t_win, [arr["force_wy"][mask]])
        self._set_plot("force_wz", t_win, [arr["force_wz"][mask]])
        self._set_plot("force_contact", t_win, [arr["force_cmd"][mask], arr["force_contact_x"][mask]])

        self._set_plot("att_roll", t_win, [arr["roll_des"][mask], arr["roll_act"][mask]])
        self._set_plot("att_pitch", t_win, [arr["pitch_des"][mask], arr["pitch_act"][mask]])
        self._set_plot("att_yaw", t_win, [arr["yaw_des"][mask], arr["yaw_act"][mask]])

        self._set_plot("motor", t_win, [arr["m1"][mask], arr["m2"][mask], arr["m3"][mask], arr["m4"][mask]])

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
            f"att=({arr['roll_act'][-1]:.3f}, {arr['pitch_act'][-1]:.3f}, {arr['yaw_act'][-1]:.3f})   "
            f"Fcmd={arr['force_cmd'][-1]:.4f}   "
            f"Fcx={arr['force_contact_x'][-1]:.4f}   "
            f"Fw=({arr['force_wx'][-1]:.4f}, {arr['force_wy'][-1]:.4f}, {arr['force_wz'][-1]:.4f})   "
            f"m=[{arr['m1'][-1]:.3f}, {arr['m2'][-1]:.3f}, {arr['m3'][-1]:.3f}, {arr['m4'][-1]:.3f}]"
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