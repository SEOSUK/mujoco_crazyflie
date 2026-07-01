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
    QFrame,
    QGridLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray


AXES = ("x", "y", "z")
DISPLAY_SCALE = 1.0e-3
FORCE_AXIS_UNIT = 1.0e-6
TORQUE_AXIS_UNIT = 1.0e-9
FORCE_AXIS_LIMIT = 50.0
TORQUE_AXIS_LIMIT = 3000.0


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
        super().__init__("mob_consistency_debug")

        self.declare_parameter("history_sec", 8.0)
        self.declare_parameter("update_hz", 60.0)
        self.declare_parameter("pose_topic", "/crazyflie/out/pose")
        self.declare_parameter("input_topic", "/crazyflie/in/input")
        self.declare_parameter("force_gt_topic", "/crazyflie/out/EE_contact_force_filt")
        self.declare_parameter("mob_2nd_topic", "/crazyflie/out/mob_2nd")
        self.declare_parameter("mob_consistency_topic", "/crazyflie/out/mob_2nd_tau")
        self.declare_parameter("mob_consistency_terms_topic", "/crazyflie/out/mob_2nd_tau_terms")
        self.declare_parameter("mob_consistency_match_topic", "/crazyflie/out/mob_2nd_tau_consistency")
        self.declare_parameter("end_effector_offset", [0.1, 0.0, 0.04])
        self.declare_parameter("contact_force_threshold", 0.005)
        self.declare_parameter("epsilon_tau_abs", 0.001)
        self.declare_parameter("mass", 0.04338)
        self.declare_parameter("g", 9.81)

        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.contact_force_threshold = float(self.get_parameter("contact_force_threshold").value)
        self.epsilon_tau_abs = float(self.get_parameter("epsilon_tau_abs").value)
        self.mass = float(self.get_parameter("mass").value)
        self.gravity = float(self.get_parameter("g").value)

        ee_offset = list(self.get_parameter("end_effector_offset").value)
        if len(ee_offset) != 3:
            ee_offset = [0.1, 0.0, 0.04]
        self.ee_offset_body = np.array(ee_offset, dtype=float)

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(120, int(self.history_sec * self.update_hz) + 20)

        keys = [
            "gt_fx", "gt_fy", "gt_fz", "gt_f_norm",
            "ctrl_fx", "ctrl_fy", "ctrl_fz", "ctrl_f_norm",
            "mob1_fx", "mob1_fy", "mob1_fz", "mob1_f_norm",
            "mob2_fx", "mob2_fy", "mob2_fz", "mob2_f_norm",
            "mobc_fx", "mobc_fy", "mobc_fz", "mobc_f_norm",
            "err2_fx", "err2_fy", "err2_fz", "err2_f_norm",
            "errc_fx", "errc_fy", "errc_fz", "errc_f_norm",
            "mob1_tx", "mob1_ty", "mob1_tz", "mob1_t_norm",
            "tau2_tx", "tau2_ty", "tau2_tz", "tau2_norm",
            "ctrl_tau_tx", "ctrl_tau_ty", "ctrl_tau_tz", "ctrl_tau_norm",
            "tauc_tx", "tauc_ty", "tauc_tz", "tauc_norm",
            "tauhat_tx", "tauhat_ty", "tauhat_tz", "tauhat_norm",
            "rxf_tx", "rxf_ty", "rxf_tz", "rxf_norm",
            "rxf_mob2_tx", "rxf_mob2_ty", "rxf_mob2_tz", "rxf_mob2_norm",
            "rxf_mobc_tx", "rxf_mobc_ty", "rxf_mobc_tz", "rxf_mobc_norm",
            "rxf_gt_tx", "rxf_gt_ty", "rxf_gt_tz", "rxf_gt_norm",
            "e_tau_x", "e_tau_y", "e_tau_z", "e_tau_norm",
            "e_tau_rel", "tau_ok",
            "base_update_norm", "consistency_update_norm", "update_ratio",
            "rmse_force_2nd", "rmse_force_consistency", "force_improvement_pct",
            "rmse_tau_residual", "contact_active",
        ]

        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for key in keys:
            self.data[key] = deque(maxlen=self.maxlen)

        self.latest = {key: math.nan for key in keys}
        self.latest["tau_ok"] = 0.0
        self.latest["contact_active"] = 0.0

        self.force_gt = nan_vec()
        self.ctrl_force_world = nan_vec()
        self.ctrl_tau_world = nan_vec()
        self.input_tau_fz = np.full(4, math.nan, dtype=float)
        self.mob1_force = nan_vec()
        self.mob1_torque = nan_vec()
        self.mob2_force = nan_vec()
        self.mob2_torque = nan_vec()
        self.mobc_force = nan_vec()
        self.mobc_torque = nan_vec()
        self.base_update = nan_vec()
        self.consistency_update = nan_vec()
        self.tauhat_world = nan_vec()
        self.rxf_world = nan_vec()
        self.rot_body_to_world = np.eye(3, dtype=float)
        self.r_world = self.ee_offset_body.copy()

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
            WrenchStamped,
            str(self.get_parameter("force_gt_topic").value),
            self.cb_force_gt,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("mob_2nd_topic").value),
            self.cb_mob_2nd,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("mob_consistency_topic").value),
            self.cb_mob_consistency,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("mob_consistency_terms_topic").value),
            self.cb_mob_terms,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("mob_consistency_match_topic").value),
            self.cb_mob_match,
            10,
        )
        self.timer = self.create_timer(1.0 / self.update_hz, self.log_snapshot)
        self.get_logger().info("MOB_consistency debug panel started")

    def cb_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        rot = quat_xyzw_to_rotmat(q.x, q.y, q.z, q.w)
        with self.lock:
            self.rot_body_to_world = rot
            self.r_world = rot @ self.ee_offset_body

    def cb_input(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        tau_fz = np.array(msg.data[:4], dtype=float)
        with self.lock:
            self.input_tau_fz = tau_fz
            self.ctrl_tau_world = self.rot_body_to_world @ tau_fz[:3]
            self.ctrl_force_world = self.rot_body_to_world @ np.array([0.0, 0.0, tau_fz[3]], dtype=float)
            self.ctrl_force_world[2] -= self.mass * self.gravity

    def cb_force_gt(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.force_gt = wrench_force(msg)

    def cb_mob_2nd(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.mob2_force = wrench_force(msg)
            self.mob2_torque = wrench_torque(msg)

    def cb_mob_consistency(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.mobc_force = wrench_force(msg)
            self.mobc_torque = wrench_torque(msg)

    def cb_mob_terms(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.base_update = wrench_force(msg)
            self.consistency_update = wrench_torque(msg)

    def cb_mob_match(self, msg: WrenchStamped) -> None:
        with self.lock:
            self.tauhat_world = wrench_force(msg)
            self.rxf_world = wrench_torque(msg)

    def _window_rmse_locked(self, key: str, contact_only: bool = True) -> float:
        values = np.array(self.data[key], dtype=float)
        if values.size == 0:
            return math.nan

        mask = np.isfinite(values)
        if contact_only:
            contact = np.array(self.data["contact_active"], dtype=float) > 0.5
            if contact.size == values.size:
                mask = mask & contact

        if not np.any(mask):
            return math.nan
        return float(np.sqrt(np.mean(values[mask] * values[mask])))

    def log_snapshot(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0

        with self.lock:
            f_gt = self.force_gt.copy()
            f1 = self.mob1_force.copy()
            f2 = self.mob2_force.copy()
            fc = self.mobc_force.copy()
            tau1 = self.mob1_torque.copy()
            tau2 = self.mob2_torque.copy()
            tauc = self.mobc_torque.copy()
            base = self.base_update.copy()
            corr = self.consistency_update.copy()
            tauhat = self.tauhat_world.copy()
            rxf = self.rxf_world.copy()
            r_world = self.r_world.copy()
            rot = self.rot_body_to_world.copy()
            tau_fz = self.input_tau_fz.copy()

            if np.all(np.isfinite(tau_fz)):
                f_ctrl = rot @ np.array([0.0, 0.0, tau_fz[3]], dtype=float)
                f_ctrl[2] -= self.mass * self.gravity
                tau_ctrl = rot @ tau_fz[:3]
            else:
                f_ctrl = nan_vec()
                tau_ctrl = nan_vec()

            err2 = f2 - f_gt
            errc = fc - f_gt
            e_tau = tauhat - rxf
            rxf_mob2 = np.cross(r_world, f2) if np.all(np.isfinite(r_world)) else nan_vec()
            rxf_mobc = np.cross(r_world, fc) if np.all(np.isfinite(r_world)) else nan_vec()
            rxf_gt = np.cross(r_world, f_gt) if np.all(np.isfinite(r_world)) else nan_vec()

            f_gt_norm = finite_norm(f_gt)
            f_ctrl_norm = finite_norm(f_ctrl)
            f1_norm = finite_norm(f1)
            f2_norm = finite_norm(f2)
            fc_norm = finite_norm(fc)
            err2_norm = finite_norm(err2)
            errc_norm = finite_norm(errc)
            tau1_norm = finite_norm(tau1)
            tau2_norm = finite_norm(tau2)
            tau_ctrl_norm = finite_norm(tau_ctrl)
            tauc_norm = finite_norm(tauc)
            tauhat_norm = finite_norm(tauhat)
            rxf_norm = finite_norm(rxf)
            rxf_mob2_norm = finite_norm(rxf_mob2)
            rxf_mobc_norm = finite_norm(rxf_mobc)
            rxf_gt_norm = finite_norm(rxf_gt)
            e_tau_norm = finite_norm(e_tau)
            base_norm = finite_norm(base)
            corr_norm = finite_norm(corr)

            e_tau_rel = math.nan
            if math.isfinite(e_tau_norm):
                denom = max(finite_norm(r_world) * max(fc_norm if math.isfinite(fc_norm) else 0.0, 0.0), 1e-9)
                e_tau_rel = e_tau_norm / denom

            update_ratio = math.nan
            if math.isfinite(base_norm) and math.isfinite(corr_norm):
                update_ratio = corr_norm / (base_norm + 1e-9)

            contact_active = 1.0 if math.isfinite(f_gt_norm) and f_gt_norm >= self.contact_force_threshold else 0.0
            tau_ok = 1.0 if math.isfinite(e_tau_norm) and e_tau_norm <= self.epsilon_tau_abs else 0.0

            snapshot = {
                "gt_fx": f_gt[0], "gt_fy": f_gt[1], "gt_fz": f_gt[2], "gt_f_norm": f_gt_norm,
                "ctrl_fx": f_ctrl[0], "ctrl_fy": f_ctrl[1], "ctrl_fz": f_ctrl[2], "ctrl_f_norm": f_ctrl_norm,
                "mob1_fx": f1[0], "mob1_fy": f1[1], "mob1_fz": f1[2], "mob1_f_norm": f1_norm,
                "mob2_fx": f2[0], "mob2_fy": f2[1], "mob2_fz": f2[2], "mob2_f_norm": f2_norm,
                "mobc_fx": fc[0], "mobc_fy": fc[1], "mobc_fz": fc[2], "mobc_f_norm": fc_norm,
                "err2_fx": err2[0], "err2_fy": err2[1], "err2_fz": err2[2], "err2_f_norm": err2_norm,
                "errc_fx": errc[0], "errc_fy": errc[1], "errc_fz": errc[2], "errc_f_norm": errc_norm,
                "mob1_tx": tau1[0], "mob1_ty": tau1[1], "mob1_tz": tau1[2], "mob1_t_norm": tau1_norm,
                "tau2_tx": tau2[0], "tau2_ty": tau2[1], "tau2_tz": tau2[2], "tau2_norm": tau2_norm,
                "ctrl_tau_tx": tau_ctrl[0], "ctrl_tau_ty": tau_ctrl[1], "ctrl_tau_tz": tau_ctrl[2],
                "ctrl_tau_norm": tau_ctrl_norm,
                "tauc_tx": tauc[0], "tauc_ty": tauc[1], "tauc_tz": tauc[2], "tauc_norm": tauc_norm,
                "tauhat_tx": tauhat[0], "tauhat_ty": tauhat[1], "tauhat_tz": tauhat[2], "tauhat_norm": tauhat_norm,
                "rxf_tx": rxf[0], "rxf_ty": rxf[1], "rxf_tz": rxf[2], "rxf_norm": rxf_norm,
                "rxf_mob2_tx": rxf_mob2[0], "rxf_mob2_ty": rxf_mob2[1], "rxf_mob2_tz": rxf_mob2[2],
                "rxf_mob2_norm": rxf_mob2_norm,
                "rxf_mobc_tx": rxf_mobc[0], "rxf_mobc_ty": rxf_mobc[1], "rxf_mobc_tz": rxf_mobc[2],
                "rxf_mobc_norm": rxf_mobc_norm,
                "rxf_gt_tx": rxf_gt[0], "rxf_gt_ty": rxf_gt[1], "rxf_gt_tz": rxf_gt[2], "rxf_gt_norm": rxf_gt_norm,
                "e_tau_x": e_tau[0], "e_tau_y": e_tau[1], "e_tau_z": e_tau[2], "e_tau_norm": e_tau_norm,
                "e_tau_rel": e_tau_rel, "tau_ok": tau_ok,
                "base_update_norm": base_norm, "consistency_update_norm": corr_norm,
                "update_ratio": update_ratio,
                "contact_active": contact_active,
            }

            self.data["t"].append(t)
            for key, value in snapshot.items():
                self.latest[key] = float(value)
                self.data[key].append(float(value))

            rmse2 = self._window_rmse_locked("err2_f_norm", contact_only=True)
            rmsec = self._window_rmse_locked("errc_f_norm", contact_only=True)
            rmse_tau = self._window_rmse_locked("e_tau_norm", contact_only=True)

            improvement = math.nan
            if math.isfinite(rmse2) and abs(rmse2) > 1e-12 and math.isfinite(rmsec):
                improvement = 100.0 * (rmse2 - rmsec) / rmse2

            for key, value in {
                "rmse_force_2nd": rmse2,
                "rmse_force_consistency": rmsec,
                "force_improvement_pct": improvement,
                "rmse_tau_residual": rmse_tau,
            }.items():
                self.latest[key] = float(value)
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
        self.setWindowTitle("MOB_consistency")

        pg.setConfigOptions(antialias=True)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        self.info_label = QLabel("Waiting for MOB consistency data...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.info_label.setWordWrap(True)
        self.info_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        root.addWidget(self.info_label)

        root.addWidget(self._build_force_panel(), stretch=3)
        root.addWidget(self._build_torque_panel(), stretch=3)
        root.addWidget(self._build_consistency_match_panel(), stretch=2)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(100)

        self.resize(760, 1280)
        self.setFixedWidth(760)

    def _make_group_frame(self, title: str) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setLineWidth(1)
        frame.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        label.setStyleSheet("""
            QLabel {
                font-size: 15px;
                font-weight: bold;
                padding: 2px;
            }
        """)
        layout.addWidget(label)
        return frame

    def _make_plot(self, title: str, ylabel: str, min_height: int = 105, y_range=None):
        plot = pg.PlotWidget()
        plot.setBackground("w")
        plot.setMinimumWidth(72)
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
        if y_range is not None:
            pi.enableAutoRange(axis="y", enable=False)
            pi.setYRange(float(y_range[0]), float(y_range[1]), padding=0.0)
        legend = pi.addLegend()
        legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))
        return plot

    def _build_force_panel(self) -> QWidget:
        frame = self._make_group_frame("Force Channel")
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.force_plots = []
        self.force_curves = {}
        colors = {
            "gt": pg.mkPen((20, 20, 20), width=2, style=Qt.DashLine),
            "mob2": pg.mkPen((40, 90, 220), width=2),
            "mobc": pg.mkPen((220, 70, 45), width=2),
        }

        for i, axis in enumerate(AXES):
            plot = self._make_plot(
                f"F_{axis}",
                "force [N] x1e-3 (axis x1e-06)",
                y_range=(-FORCE_AXIS_LIMIT, FORCE_AXIS_LIMIT),
            )
            self.force_plots.append(plot)
            self.force_curves[f"gt_f{axis}"] = plot.plot(name="GT contact", pen=colors["gt"])
            self.force_curves[f"mob2_f{axis}"] = plot.plot(name="MOB 2nd", pen=colors["mob2"])
            self.force_curves[f"mobc_f{axis}"] = plot.plot(name="MOB consistency", pen=colors["mobc"])
            grid.addWidget(plot, 0, i)

        frame.layout().addLayout(grid)
        return frame

    def _build_error_panel(self) -> QWidget:
        frame = self._make_group_frame("Force Error and Rolling Score")
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.plot_force_err = self._make_plot(
            "||F_hat - F_gt||",
            "force error [N] x1e-3 (axis x1e-06)",
            min_height=145,
            y_range=(0.0, FORCE_AXIS_LIMIT),
        )
        self.curve_err2 = self.plot_force_err.plot(name="MOB 2nd", pen=pg.mkPen((40, 90, 220), width=2))
        self.curve_errc = self.plot_force_err.plot(name="MOB consistency", pen=pg.mkPen((220, 70, 45), width=2))

        self.plot_rmse = self._make_plot(
            "rolling contact RMSE",
            "RMSE [N] x1e-3 (axis x1e-06)",
            min_height=145,
            y_range=(0.0, FORCE_AXIS_LIMIT),
        )
        self.curve_rmse2 = self.plot_rmse.plot(name="MOB 2nd", pen=pg.mkPen((40, 90, 220), width=2))
        self.curve_rmsec = self.plot_rmse.plot(name="MOB consistency", pen=pg.mkPen((220, 70, 45), width=2))

        self.plot_improve = self._make_plot("force RMSE improvement", "improvement [%]", min_height=145)
        self.curve_improve = self.plot_improve.plot(name="positive is better", pen=pg.mkPen((40, 150, 75), width=2))
        self.plot_improve.setYRange(-100.0, 100.0, padding=0.0)

        grid.addWidget(self.plot_force_err, 0, 0)
        grid.addWidget(self.plot_rmse, 0, 1)
        grid.addWidget(self.plot_improve, 0, 2)
        frame.layout().addLayout(grid)
        return frame

    def _build_torque_panel(self) -> QWidget:
        frame = self._make_group_frame("Torque Channel")
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.torque_plots = []
        self.torque_curves = {}
        colors = {
            "ctrl": pg.mkPen((40, 150, 75), width=2),
            "rxf_gt": pg.mkPen((20, 20, 20), width=2, style=Qt.DashLine),
            "tau2": pg.mkPen((40, 90, 220), width=2),
        }

        for i, axis in enumerate(AXES):
            plot = self._make_plot(
                f"tau_{axis}",
                "torque [Nm] x1e-3 (axis x1e-09)",
                y_range=(-TORQUE_AXIS_LIMIT, TORQUE_AXIS_LIMIT),
            )
            self.torque_plots.append(plot)
            self.torque_curves[f"ctrl_tau_t{axis}"] = plot.plot(name="control tau", pen=colors["ctrl"])
            self.torque_curves[f"rxf_gt_t{axis}"] = plot.plot(name="GT r x F_gt", pen=colors["rxf_gt"])
            self.torque_curves[f"tau2_t{axis}"] = plot.plot(name="MOB 2nd torque", pen=colors["tau2"])
            self.torque_curves[f"tau2_t{axis}"].setZValue(1)
            self.torque_curves[f"rxf_gt_t{axis}"].setZValue(2)
            self.torque_curves[f"ctrl_tau_t{axis}"].setZValue(3)
            grid.addWidget(plot, 0, i)

        frame.layout().addLayout(grid)
        return frame

    def _build_consistency_match_panel(self) -> QWidget:
        frame = self._make_group_frame("Consistency Residual: tau_hat_ext vs r x F")
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.consistency_match_plots = []
        self.consistency_match_curves = {}
        colors = {
            "tauhat": pg.mkPen((40, 90, 220), width=2),
            "rxf": pg.mkPen((220, 70, 45), width=2),
        }

        for i, axis in enumerate(AXES):
            plot = self._make_plot(
                f"tau match {axis}",
                "torque [Nm] x1e-3 (axis x1e-09)",
                min_height=145,
                y_range=(-TORQUE_AXIS_LIMIT, TORQUE_AXIS_LIMIT),
            )
            self.consistency_match_plots.append(plot)
            self.consistency_match_curves[f"tauhat_t{axis}"] = plot.plot(
                name="tau_hat_ext", pen=colors["tauhat"])
            self.consistency_match_curves[f"rxf_t{axis}"] = plot.plot(
                name="r x F", pen=colors["rxf"])
            grid.addWidget(plot, 0, i)

        frame.layout().addLayout(grid)
        return frame

    def _fmt(self, value: float, precision: int = 4) -> str:
        if not math.isfinite(float(value)):
            return "--"
        return f"{float(value):.{precision}f}"

    def _scaled(self, values: np.ndarray) -> np.ndarray:
        return DISPLAY_SCALE * values

    def _force_axis(self, values: np.ndarray) -> np.ndarray:
        return DISPLAY_SCALE * values / FORCE_AXIS_UNIT

    def _torque_axis(self, values: np.ndarray) -> np.ndarray:
        return DISPLAY_SCALE * values / TORQUE_AXIS_UNIT

    def _fmt_scaled(self, value: float, precision: int = 6) -> str:
        if not math.isfinite(float(value)):
            return "--"
        return f"{DISPLAY_SCALE * float(value):.{precision}f}"

    def _fmt_force_axis(self, value: float, precision: int = 2) -> str:
        if not math.isfinite(float(value)):
            return "--"
        return f"{DISPLAY_SCALE * float(value) / FORCE_AXIS_UNIT:.{precision}f}"

    def _fmt_torque_axis(self, value: float, precision: int = 2) -> str:
        if not math.isfinite(float(value)):
            return "--"
        return f"{DISPLAY_SCALE * float(value) / TORQUE_AXIS_UNIT:.{precision}f}"

    def _fmt_torque_threshold_value(self) -> float:
        return DISPLAY_SCALE * float(self.rosbuf.epsilon_tau_abs) / TORQUE_AXIS_UNIT

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

        for axis in AXES:
            self.force_curves[f"gt_f{axis}"].setData(t_win, self._force_axis(arr[f"gt_f{axis}"][mask]))
            self.force_curves[f"mob2_f{axis}"].setData(t_win, self._force_axis(arr[f"mob2_f{axis}"][mask]))
            self.force_curves[f"mobc_f{axis}"].setData(t_win, self._force_axis(arr[f"mobc_f{axis}"][mask]))

            self.torque_curves[f"ctrl_tau_t{axis}"].setData(t_win, self._torque_axis(arr[f"ctrl_tau_t{axis}"][mask]))
            self.torque_curves[f"rxf_gt_t{axis}"].setData(t_win, self._torque_axis(arr[f"rxf_gt_t{axis}"][mask]))
            self.torque_curves[f"tau2_t{axis}"].setData(t_win, self._torque_axis(arr[f"tau2_t{axis}"][mask]))

            self.consistency_match_curves[f"tauhat_t{axis}"].setData(
                t_win, self._torque_axis(arr[f"tauhat_t{axis}"][mask]))
            self.consistency_match_curves[f"rxf_t{axis}"].setData(
                t_win, self._torque_axis(arr[f"rxf_t{axis}"][mask]))

        x_left = max(0.0, tmax - window)
        x_right = tmax if tmax >= window else window
        plots = [
            *self.force_plots,
            *self.torque_plots,
            *self.consistency_match_plots,
        ]
        for plot in plots:
            plot.setXRange(x_left, x_right, padding=0.0)

        contact = "ON" if latest["contact_active"] > 0.5 else "off"
        tau_ok = "OK" if latest["tau_ok"] > 0.5 else "HIGH"
        self.info_label.setText(
            f"contact={contact}   "
            f"torque_frame=world   "
            f"|F_gt|={self._fmt_force_axis(latest['gt_f_norm'])} N(x1e-3/x1e-06)   "
            f"|F_ctrl|={self._fmt_force_axis(latest['ctrl_f_norm'])} N(x1e-3/x1e-06)   "
            f"|tau_ctrl|={self._fmt_torque_axis(latest['ctrl_tau_norm'])} Nm(x1e-3/x1e-09)"
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
