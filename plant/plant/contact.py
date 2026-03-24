#!/usr/bin/env python3
import numpy as np
import mujoco
from geometry_msgs.msg import WrenchStamped


class ContactManager:
    def __init__(self, node, model, data, pub_contact_force, pub_contact_force_filt):
        self.node = node
        self.model = model
        self.data = data
        self.pub_contact_force = pub_contact_force
        self.pub_contact_force_filt = pub_contact_force_filt

        # ---- Contact arrow viz params ----
        self.node.declare_parameter("viz.contact_arrows.enable", True)
        self.node.declare_parameter("viz.contact_arrows.scale", 4.5)
        self.node.declare_parameter("viz.contact_arrows.width", 0.008)
        self.node.declare_parameter("viz.contact_arrows.max", 64)

        self.viz_contact_enable = bool(self.node.get_parameter("viz.contact_arrows.enable").value)
        self.viz_contact_scale = float(self.node.get_parameter("viz.contact_arrows.scale").value)
        self.viz_contact_width = float(self.node.get_parameter("viz.contact_arrows.width").value)
        self.viz_contact_max = int(self.node.get_parameter("viz.contact_arrows.max").value)

        # 6D force buffer for mj_contactForce
        self._cf6 = np.zeros(6, dtype=float)

        # ---- contact filter params ----
        self.node.declare_parameter("contact_filter.enable", True)
        self.node.declare_parameter("contact_filter.cutoff_hz", 5.0)
        self.node.declare_parameter("contact_filter.timer_hz", 100.0)
        self.node.declare_parameter("contact_filter.use_exp_alpha", True)

        self.contact_filter_enable = bool(self.node.get_parameter("contact_filter.enable").value)
        self.contact_cutoff_hz = float(self.node.get_parameter("contact_filter.cutoff_hz").value)
        self.contact_timer_hz = float(self.node.get_parameter("contact_filter.timer_hz").value)
        self.use_exp_alpha = bool(self.node.get_parameter("contact_filter.use_exp_alpha").value)

        # raw force latest
        self._F_raw_latest = np.zeros(3, dtype=float)

        # diff/filter states
        self._F_raw_prev = np.zeros(3, dtype=float)
        self._F_filt_latest = np.zeros(3, dtype=float)
        self._Fdot_raw_latest = np.zeros(3, dtype=float)
        self._Fdot_filt_latest = np.zeros(3, dtype=float)

        self._contact_timer_prev_ns = None

        # contact resultants
        self.fcn = 0.0
        self.rf = np.zeros(3, dtype=float)
        self.Fw = np.zeros(3, dtype=float)

        # geom ids
        self.gid_tip = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ee_tip_sphere")
        self.gid_box = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wall_collision")

        if self.gid_tip < 0 or self.gid_box < 0:
            raise RuntimeError(
                f"geom id not found: tip={self.gid_tip}, box={self.gid_box}. "
                "Check geom names in XML."
            )

        self._last_contact_log_ns = 0

        self.timer_contact = None
        if self.contact_filter_enable:
            period = 1.0 / self.contact_timer_hz
            self.timer_contact = self.node.create_timer(period, self.cb_contact_diff_100hz)

    def update_raw_and_publish(self, stamp):
        self.mjfc(self.model, self.data)
        F_raw = self.Fw.copy()

        self._F_raw_latest = F_raw.copy()

        cf_msg = WrenchStamped()
        cf_msg.header.stamp = stamp
        cf_msg.header.frame_id = "world"
        cf_msg.wrench.force.x = float(F_raw[0])
        cf_msg.wrench.force.y = float(F_raw[1])
        cf_msg.wrench.force.z = float(F_raw[2])
        cf_msg.wrench.torque.x = 0.0
        cf_msg.wrench.torque.y = 0.0
        cf_msg.wrench.torque.z = 0.0
        self.pub_contact_force.publish(cf_msg)

    def cb_contact_diff_100hz(self):
        now_ns = self.node.get_clock().now().nanoseconds

        if self._contact_timer_prev_ns is None:
            self._contact_timer_prev_ns = now_ns
            self._F_raw_prev = self._F_raw_latest.copy()
            return

        dt = (now_ns - self._contact_timer_prev_ns) * 1e-9
        self._contact_timer_prev_ns = now_ns

        if dt <= 1e-6 or dt > 0.05:
            return

        F = self._F_raw_latest
        Fdot_raw = (F - self._F_raw_prev) / dt
        self._F_raw_prev = F.copy()

        self._Fdot_raw_latest = Fdot_raw.copy()

        wc = float(max(0.0, self.contact_cutoff_hz))
        if wc <= 0.0:
            a = 1.0
        else:
            a = float(np.clip(wc * dt, 0.0, 1.0))

        self._Fdot_filt_latest = (1.0 - a) * self._Fdot_filt_latest + a * Fdot_raw
        self._F_filt_latest = (1.0 - a) * self._F_filt_latest + a * F

        msg = WrenchStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.wrench.force.x = float(self._F_filt_latest[0])
        msg.wrench.force.y = float(self._F_filt_latest[1])
        msg.wrench.force.z = float(self._F_filt_latest[2])
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.pub_contact_force_filt.publish(msg)

    @staticmethod
    def _contact_frame_to_world(con) -> np.ndarray:
        fr = np.array(con.frame, dtype=float).ravel()
        if fr.size == 9:
            return fr.reshape(3, 3)
        return np.eye(3, dtype=float)

    @staticmethod
    def _arrow_mat_from_dir(d: np.ndarray) -> np.ndarray:
        z = d / (np.linalg.norm(d) + 1e-12)

        up = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(np.dot(z, up)) > 0.95:
            up = np.array([0.0, 1.0, 0.0], dtype=float)

        x = np.cross(up, z)
        x = x / (np.linalg.norm(x) + 1e-12)
        y = np.cross(z, x)

        return np.column_stack([x, y, z])

    @staticmethod
    def _set_geom_mat(g, R: np.ndarray):
        try:
            if getattr(g.mat, "shape", None) == (3, 3):
                g.mat[:, :] = R
            else:
                g.mat[:] = R.reshape(-1)
        except Exception:
            pass

    def mjfc(self, model, data):
        self.fcn = 0.0
        self.rf = np.zeros(3, dtype=float)
        self.Fw = np.zeros(3, dtype=float)

        now_ns = self.node.get_clock().now().nanoseconds
        do_log = False
        if now_ns - self._last_contact_log_ns > 1_000_000_000:
            self._last_contact_log_ns = now_ns
            do_log = True

        for i in range(data.ncon):
            con = data.contact[i]

            if do_log and i < 5:
                _ = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(con.geom1))
                _ = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(con.geom2))

            g1 = int(con.geom1)
            g2 = int(con.geom2)
            if not ((g1 == self.gid_tip and g2 == self.gid_box) or
                    (g1 == self.gid_box and g2 == self.gid_tip)):
                continue

            fci = np.zeros(6, dtype=float)
            try:
                mujoco.mj_contactForce(model, data, i, fci)

                pos_w = np.array(con.pos, dtype=float)
                R = self._contact_frame_to_world(con)

                n_w = R[0, :]
                t1_w = R[1, :]
                t2_w = R[2, :]

                F_c = fci[0:3].copy()
                C = np.column_stack([n_w, t1_w, t2_w])
                F_w = C @ F_c

                Fn_mag = float(np.linalg.norm(F_w))
                if Fn_mag < 1e-12:
                    continue

                self.fcn += Fn_mag
                self.rf += pos_w * Fn_mag
                self.Fw += F_w

            except Exception:
                pass

    def update_contact_resultant_arrow_in_viewer(self, viewer):
        if (not self.viz_contact_enable) or self.viz_contact_scale <= 0.0:
            viewer.user_scn.ngeom = 0
            return

        viewer.user_scn.ngeom = 0

        if self.fcn <= 1e-12:
            return

        p0 = self.rf / self.fcn
        F = self.Fw
        Fn = float(np.linalg.norm(F))
        if Fn <= 1e-12:
            return

        d = F / Fn
        length = self.viz_contact_scale * Fn

        g = viewer.user_scn.geoms[0]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3),
            np.zeros(3),
            np.zeros(9),
            np.array([1, 0, 0, 1], dtype=float)
        )

        g.pos[:] = p0
        R = self._arrow_mat_from_dir(d)
        self._set_geom_mat(g, R)
        g.size[:] = np.array([self.viz_contact_width, self.viz_contact_width, length], dtype=float)

        viewer.user_scn.ngeom = 1

    def close(self):
        pass