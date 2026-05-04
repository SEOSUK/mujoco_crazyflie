#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import signal
import sys
import threading
from collections import deque
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET

import numpy as np

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

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

from geometry_msgs.msg import PoseStamped, QuaternionStamped, Vector3Stamped, WrenchStamped
from std_msgs.msg import Float32, Float64MultiArray


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


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na < 1e-12 or nb < 1e-12:
        return math.nan
    cosine = float(np.dot(a, b) / (na * nb))
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def safe_unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.full_like(v, math.nan, dtype=float)
    return v / n


def velocity_evidence_normal(l_v: np.ndarray) -> np.ndarray:
    if not np.all(np.isfinite(l_v)):
        return np.full(3, math.nan, dtype=float)

    mat = 0.5 * (l_v + l_v.T)
    if float(np.linalg.norm(mat, ord="fro")) < 1e-12:
        return np.full(3, math.nan, dtype=float)

    try:
        values, vectors = np.linalg.eigh(mat)
    except np.linalg.LinAlgError:
        return np.full(3, math.nan, dtype=float)

    idx = int(np.argmin(values))
    return safe_unit(vectors[:, idx])


def load_cylinder_geometry() -> Tuple[np.ndarray, float]:
    for package_name in ("mujoco_bridge", "plant"):
        try:
            share = get_package_share_directory(package_name)
        except Exception:
            continue

        cylinder_path = os.path.join(share, "data", "cylinder.xml")
        if not os.path.exists(cylinder_path):
            continue

        try:
            root = ET.parse(cylinder_path).getroot()
            body = root.find(".//body[@name='wall']")
            geom = root.find(".//geom[@name='cylinder_collision']")
            if body is None or geom is None:
                continue

            body_pos = np.array([float(v) for v in body.attrib.get("pos", "1.5 0 0").split()], dtype=float)
            size_vals = [float(v) for v in geom.attrib.get("size", "1 1").split()]
            radius = float(size_vals[0]) if len(size_vals) >= 1 else 1.0
            return body_pos, radius
        except Exception:
            continue

    return np.array([1.5, 0.0, 0.0], dtype=float), 1.0


class ROSDataBuffer(Node):
    def __init__(self):
        super().__init__("normal_vector_lvlf")

        self.declare_parameter("history_sec", 3.0)
        self.declare_parameter("update_hz", 30.0)
        self.declare_parameter("render_hz", 3.0)
        self.declare_parameter("contact_quat_topic", "/estimated_contact_frame_quat")
        self.declare_parameter("ee_pose_topic", "/crazyflie/out/EE_pose")
        self.declare_parameter("vel_cmd_topic", "/su/debug/contact_vel_cmd")
        self.declare_parameter("vel_actual_topic", "/su/debug/contact_vel_actual")
        self.declare_parameter("force_cmd_topic", "su/cmd_force")
        self.declare_parameter("force_actual_topic", "/su/contact_force_x")
        self.declare_parameter("force_world_topic", "/crazyflie/out/EE_contact_force_filt")
        self.declare_parameter("consistency_topic", "/crazyflie/out/mob_2nd_tau_consistency")
        self.declare_parameter("reference_normal_topic", "")
        self.declare_parameter("packed_debug_topic", "/normal_vector/debug_metrics")
        self.declare_parameter("eta_tau", 0.4)
        self.declare_parameter("sigma_tau", 0.004)
        self.declare_parameter("environment.type", "cylinder")
        self.declare_parameter("cylinder.pos.x", 1.5)
        self.declare_parameter("cylinder.pos.y", 0.0)
        self.declare_parameter("cylinder.pos.z", 0.0)
        self.declare_parameter("cylinder.radius", 1.0)
        self.declare_parameter("wall.pos.x", 0.2)

        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.render_hz = max(0.1, float(self.get_parameter("render_hz").value))
        self.eta_tau = float(self.get_parameter("eta_tau").value)
        self.sigma_tau = float(self.get_parameter("sigma_tau").value)
        self.environment_type = str(self.get_parameter("environment.type").value).lower()
        if self.environment_type not in ("wall", "cylinder"):
            self.environment_type = "cylinder"

        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.maxlen = max(80, int(self.history_sec * self.update_hz) + 4)

        keys = [
            "c_hat_vy_cmd", "c_hat_vy_act",
            "c_hat_vz_cmd", "c_hat_vz_act",
            "c_hat_fx_cmd", "c_hat_fx_act",
            "lambda1", "lambda2", "lambda3",
            "m_tau", "e_tau_norm", "vel_norm", "force_norm",
            "rho_v", "rho_f",
            "alpha_v", "alpha_f",
            "tr_l_v", "tr_l_f",
            "angle_n_geo_deg", "angle_n_f_deg", "angle_n_v_deg", "angle_force_dir_deg",
        ]
        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for key in keys:
            self.data[key] = deque(maxlen=self.maxlen)

        self.latest: Dict[str, float] = {key: math.nan for key in keys}
        self.latest["m_tau"] = 1.0

        self.l_v_bar = np.full((3, 3), math.nan, dtype=float)
        self.l_f_bar = np.full((3, 3), math.nan, dtype=float)
        self.l_v = np.full((3, 3), math.nan, dtype=float)
        self.l_f = np.full((3, 3), math.nan, dtype=float)
        self.n_est = np.full(3, math.nan, dtype=float)
        self.n_geo = np.full(3, math.nan, dtype=float)
        self.n_f = np.full(3, math.nan, dtype=float)
        self.n_v = np.full(3, math.nan, dtype=float)
        self.n_gt = np.full(3, math.nan, dtype=float)
        self.force_dir = np.full(3, math.nan, dtype=float)
        self.ee_pos = np.full(3, math.nan, dtype=float)
        self.cylinder_center = np.array([
            float(self.get_parameter("cylinder.pos.x").value),
            float(self.get_parameter("cylinder.pos.y").value),
            float(self.get_parameter("cylinder.pos.z").value),
        ], dtype=float)
        self.cylinder_radius = float(self.get_parameter("cylinder.radius").value)
        self.wall_x = float(self.get_parameter("wall.pos.x").value)

        self.create_subscription(
            QuaternionStamped,
            str(self.get_parameter("contact_quat_topic").value),
            self.cb_contact_quat,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("ee_pose_topic").value),
            self.cb_ee_pose,
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
            str(self.get_parameter("force_world_topic").value),
            self.cb_force_world,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("consistency_topic").value),
            self.cb_consistency,
            10,
        )

        packed_topic = str(self.get_parameter("packed_debug_topic").value)
        if packed_topic:
            self.create_subscription(Float64MultiArray, packed_topic, self.cb_packed_debug, 10)

        self.timer = self.create_timer(1.0 / self.update_hz, self.log_snapshot)
        self.get_logger().info("normal_vector_lvlf started")

    def cb_contact_quat(self, msg: QuaternionStamped) -> None:
        q = msg.quaternion
        rot = quat_xyzw_to_rotmat(q.x, q.y, q.z, q.w)
        with self.lock:
            self.n_est[:] = rot[:, 0]

    def cb_ee_pose(self, msg: PoseStamped) -> None:
        with self.lock:
            self.ee_pos[:] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self._update_gt_normal_locked()
            self._update_quality_angles_locked()

    def cb_contact_vel_cmd(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest["c_hat_vy_cmd"] = float(msg.vector.y)
            self.latest["c_hat_vz_cmd"] = float(msg.vector.z)

    def cb_contact_vel_actual(self, msg: Vector3Stamped) -> None:
        vel_norm = float(np.linalg.norm([msg.vector.x, msg.vector.y, msg.vector.z]))
        with self.lock:
            self.latest["c_hat_vy_act"] = float(msg.vector.y)
            self.latest["c_hat_vz_act"] = float(msg.vector.z)
            self.latest["vel_norm"] = vel_norm

    def cb_force_cmd(self, msg: Float32) -> None:
        with self.lock:
            self.latest["c_hat_fx_cmd"] = float(msg.data)

    def cb_force_actual(self, msg: Float32) -> None:
        with self.lock:
            self.latest["c_hat_fx_act"] = float(msg.data)

    def cb_force_world(self, msg: WrenchStamped) -> None:
        force_norm = float(np.linalg.norm([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ]))
        with self.lock:
            self.latest["force_norm"] = force_norm
            self.force_dir[:] = safe_unit(np.array([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
            ], dtype=float))
            self._update_quality_angles_locked()

    def cb_consistency(self, msg: WrenchStamped) -> None:
        # m_tau is low-pass filtered in normal_vector_estimation and published
        # through /normal_vector/debug_metrics. Keep the panel on that filtered
        # value instead of recomputing an unsmoothed one here.
        pass

    def cb_packed_debug(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 34:
            return

        with self.lock:
            self.latest["lambda1"] = float(data[0])
            self.latest["lambda2"] = float(data[1])
            self.latest["lambda3"] = float(data[2])
            self.latest["m_tau"] = float(data[3])
            if len(data) > 52:
                self.latest["e_tau_norm"] = float(data[52])
            self.latest["vel_norm"] = float(data[4])
            self.latest["force_norm"] = float(data[5])
            self.latest["rho_v"] = float(data[6])
            self.latest["rho_f"] = float(data[7])
            self.latest["alpha_v"] = float(data[8])
            self.latest["alpha_f"] = float(data[9])
            self.l_v_bar = np.array(data[10:19], dtype=float).reshape(3, 3)
            self.l_f_bar = np.array(data[19:28], dtype=float).reshape(3, 3)
            if len(data) >= 52:
                self.l_v = np.array(data[34:43], dtype=float).reshape(3, 3)
                self.l_f = np.array(data[43:52], dtype=float).reshape(3, 3)
                self.latest["tr_l_v"] = float(np.trace(self.l_v))
                self.latest["tr_l_f"] = float(np.trace(self.l_f))
            else:
                self.latest["tr_l_v"] = float(np.trace(self.l_v_bar))
                self.latest["tr_l_f"] = float(np.trace(self.l_f_bar))
            self.n_geo[:] = data[28:31]
            self.n_f[:] = data[31:34]
            self.n_v[:] = velocity_evidence_normal(self.l_v_bar)
            if np.all(np.isfinite(self.n_v)):
                if np.all(np.isfinite(self.n_gt)) and np.dot(self.n_v, self.n_gt) < 0.0:
                    self.n_v[:] = -self.n_v
                elif np.all(np.isfinite(self.n_geo)) and np.dot(self.n_v, self.n_geo) < 0.0:
                    self.n_v[:] = -self.n_v
            self._update_quality_angles_locked()

    def _update_gt_normal_locked(self) -> None:
        if not np.all(np.isfinite(self.ee_pos)):
            self.n_gt[:] = np.full(3, math.nan, dtype=float)
            return

        if self.environment_type == "wall":
            self.n_gt[:] = [1.0 if self.ee_pos[0] <= self.wall_x else -1.0, 0.0, 0.0]
            return

        radial = np.array([
            self.ee_pos[0] - self.cylinder_center[0],
            self.ee_pos[1] - self.cylinder_center[1],
            0.0,
        ], dtype=float)
        self.n_gt[:] = -safe_unit(radial)

    def _update_quality_angles_locked(self) -> None:
        self.latest["angle_n_geo_deg"] = angle_deg(self.n_geo, self.n_gt)
        self.latest["angle_n_f_deg"] = angle_deg(self.n_f, self.n_gt)
        self.latest["angle_n_v_deg"] = angle_deg(self.n_v, self.n_gt)
        self.latest["angle_force_dir_deg"] = angle_deg(self.force_dir, self.n_gt)

    def log_snapshot(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        with self.lock:
            self.data["t"].append(t)
            for key, value in self.latest.items():
                self.data[key].append(value)

    def get_arrays(self) -> Dict[str, np.ndarray]:
        with self.lock:
            return {k: np.array(v, dtype=float) for k, v in self.data.items()}

    def get_summary(self) -> Tuple[Dict[str, float], np.ndarray, np.ndarray]:
        with self.lock:
            return dict(self.latest), self.l_v_bar.copy(), self.l_f_bar.copy()


class PlotWindow(QMainWindow):
    def __init__(self, rosbuf: ROSDataBuffer):
        super().__init__()
        self.rosbuf = rosbuf
        self.setWindowTitle("normal_vector_lvlf")

        pg.setConfigOptions(antialias=True)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        self.info_label = QLabel("Waiting for normal-vector debug data...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.info_label.setWordWrap(True)
        self.info_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        root.addWidget(self.info_label)

        root.addWidget(self._build_control_panel(), stretch=0)
        root.addWidget(self._build_internal_panel(), stretch=0)
        root.addWidget(self._build_fusion_panel(), stretch=0)
        root.addWidget(self._build_quality_panel(), stretch=1)

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

    def _build_internal_panel(self) -> QWidget:
        frame = self._make_group_frame("Estimation Internal Metrics")
        outer = frame.layout()

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.plot_lambda = self._make_plot("lambda1 lambda2 lambda3", "eigen", min_height=125)
        self.curve_l1 = self.plot_lambda.plot(name="lambda1", pen=pg.mkPen((220, 50, 50), width=2))
        self.curve_l2 = self.plot_lambda.plot(name="lambda2", pen=pg.mkPen((50, 160, 70), width=2))
        self.curve_l3 = self.plot_lambda.plot(name="lambda3", pen=pg.mkPen((40, 90, 220), width=2))
        self.plot_lambda.setYRange(0.0, 1.1, padding=0.0)

        self.plot_mtau = self._make_plot("e_tau_norm", "torque [N m]", min_height=125)
        self.curve_mtau = self.plot_mtau.plot(name="e_tau_norm", pen=pg.mkPen((120, 80, 220), width=2))

        self.plot_vel = self._make_plot("|v_c|", "vel [m/s]", min_height=125)
        self.curve_vel = self.plot_vel.plot(name="|v_c|", pen=pg.mkPen((80, 180, 120), width=2))
        self.plot_vel.setYRange(0.0, 0.6, padding=0.0)

        self.plot_force = self._make_plot("|f|", "force [N]", min_height=125)
        self.curve_force = self.plot_force.plot(name="|f|", pen=pg.mkPen((200, 100, 50), width=2))
        self.plot_force.setYRange(0.0, 0.1, padding=0.0)

        grid.addWidget(self.plot_lambda, 0, 0)
        grid.addWidget(self.plot_mtau, 0, 1)
        grid.addWidget(self.plot_vel, 0, 2)
        grid.addWidget(self.plot_force, 0, 3)
        outer.addLayout(grid)
        return frame

    def _build_fusion_panel(self) -> QWidget:
        frame = self._make_group_frame("Fusion Metrics")
        outer = frame.layout()

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        self.plot_rho = self._make_plot("rho_v vs rho_f", "ratio", min_height=130)
        self.curve_rho_v = self.plot_rho.plot(name="rho_v", pen=pg.mkPen((40, 90, 220), width=2))
        self.curve_rho_f = self.plot_rho.plot(name="rho_f", pen=pg.mkPen((220, 50, 50), width=2))
        self.plot_rho.setYRange(-0.1, 1.1, padding=0.0)

        self.plot_alpha = self._make_plot("alpha_v vs alpha_f", "ratio", min_height=130)
        self.curve_alpha_v = self.plot_alpha.plot(name="alpha_v", pen=pg.mkPen((50, 160, 70), width=2))
        self.curve_alpha_f = self.plot_alpha.plot(name="alpha_f", pen=pg.mkPen((180, 120, 40), width=2))
        self.plot_alpha.setYRange(-0.1, 1.1, padding=0.0)

        self.plot_trace = self._make_plot("trace(L_v) vs trace(L_f)", "trace", min_height=130)
        self.curve_tr_l_v = self.plot_trace.plot(name="tr(L_v)", pen=pg.mkPen((40, 90, 220), width=2))
        self.curve_tr_l_f = self.plot_trace.plot(name="tr(L_f)", pen=pg.mkPen((220, 50, 50), width=2))
        self.plot_trace.setYRange(-0.1, 1.1, padding=0.0)

        grid.addWidget(self.plot_rho, 0, 0)
        grid.addWidget(self.plot_alpha, 0, 1)
        grid.addWidget(self.plot_trace, 0, 2)
        outer.addLayout(grid)
        return frame

    def _build_quality_panel(self) -> QWidget:
        frame = self._make_group_frame("Estimated Normal Quality")
        outer = frame.layout()

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        quality_min_height = 180

        self.plot_ang_geo = self._make_plot("current", "deg", min_height=quality_min_height)
        self.curve_ang_geo = self.plot_ang_geo.plot(name="n_geo", pen=pg.mkPen((40, 90, 220), width=2))
        self.plot_ang_geo.setYRange(0.0, 30.0, padding=0.0)

        self.plot_ang_f = self._make_plot("Lf only (Force only)", "deg", min_height=quality_min_height)
        self.curve_ang_f = self.plot_ang_f.plot(name="n_f", pen=pg.mkPen((220, 50, 50), width=2))
        self.plot_ang_f.setYRange(0.0, 30.0, padding=0.0)

        self.plot_ang_v = self._make_plot("Lv only (Velocity only)", "deg", min_height=quality_min_height)
        self.curve_ang_v = self.plot_ang_v.plot(name="n_v", pen=pg.mkPen((50, 160, 70), width=2))
        self.plot_ang_v.setYRange(0.0, 30.0, padding=0.0)

        self.plot_ang_force = self._make_plot("raw force based", "deg", min_height=quality_min_height)
        self.curve_ang_force = self.plot_ang_force.plot(name="force_hat", pen=pg.mkPen((180, 120, 40), width=2))
        self.plot_ang_force.setYRange(0.0, 30.0, padding=0.0)

        grid.addWidget(self.plot_ang_geo, 0, 0)
        grid.addWidget(self.plot_ang_f, 0, 1)
        grid.addWidget(self.plot_ang_v, 0, 2)
        grid.addWidget(self.plot_ang_force, 0, 3)
        outer.addLayout(grid)
        return frame

    def _fmt_scalar(self, value: float, precision: int = 3) -> str:
        if not math.isfinite(float(value)):
            return "--"
        return f"{float(value):.{precision}f}"

    def update_all(self) -> None:
        arr = self.rosbuf.get_arrays()
        latest, l_v_bar, l_f_bar = self.rosbuf.get_summary()

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

        self.curve_l1.setData(t_win, arr["lambda1"][mask])
        self.curve_l2.setData(t_win, arr["lambda2"][mask])
        self.curve_l3.setData(t_win, arr["lambda3"][mask])
        self.curve_mtau.setData(t_win, arr["e_tau_norm"][mask])
        self.curve_vel.setData(t_win, arr["vel_norm"][mask])
        self.curve_force.setData(t_win, arr["force_norm"][mask])

        self.curve_rho_v.setData(t_win, arr["rho_v"][mask])
        self.curve_rho_f.setData(t_win, arr["rho_f"][mask])
        self.curve_alpha_v.setData(t_win, arr["alpha_v"][mask])
        self.curve_alpha_f.setData(t_win, arr["alpha_f"][mask])
        self.curve_tr_l_v.setData(t_win, arr["tr_l_v"][mask])
        self.curve_tr_l_f.setData(t_win, arr["tr_l_f"][mask])

        self.curve_ang_geo.setData(t_win, arr["angle_n_geo_deg"][mask])
        self.curve_ang_f.setData(t_win, arr["angle_n_f_deg"][mask])
        self.curve_ang_v.setData(t_win, arr["angle_n_v_deg"][mask])
        self.curve_ang_force.setData(t_win, arr["angle_force_dir_deg"][mask])

        x_left = max(0.0, tmax - window)
        x_right = tmax if tmax >= window else window
        for plot in [
            self.plot_vy, self.plot_vz, self.plot_fx,
            self.plot_lambda, self.plot_mtau, self.plot_vel, self.plot_force,
            self.plot_rho, self.plot_alpha, self.plot_trace,
            self.plot_ang_geo, self.plot_ang_f, self.plot_ang_v, self.plot_ang_force,
        ]:
            plot.setXRange(x_left, x_right, padding=0.0)

        self.info_label.setText(
            f"vy(cmd/act)={self._fmt_scalar(latest['c_hat_vy_cmd'])}/{self._fmt_scalar(latest['c_hat_vy_act'])}   "
            f"vz(cmd/act)={self._fmt_scalar(latest['c_hat_vz_cmd'])}/{self._fmt_scalar(latest['c_hat_vz_act'])}   "
            f"fx(cmd/act)={self._fmt_scalar(latest['c_hat_fx_cmd'])}/{self._fmt_scalar(latest['c_hat_fx_act'])}   "
            f"rho_v={self._fmt_scalar(latest['rho_v'])}   "
            f"rho_f={self._fmt_scalar(latest['rho_f'])}   "
            f"alpha_v={self._fmt_scalar(latest['alpha_v'])}   "
            f"alpha_f={self._fmt_scalar(latest['alpha_f'])}   "
            f"env={self.rosbuf.environment_type}"
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
