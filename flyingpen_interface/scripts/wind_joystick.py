#!/usr/bin/env python3

import math
import sys
import argparse

import rclpy
from geometry_msgs.msg import Vector3Stamped
from PyQt5 import QtCore, QtGui, QtWidgets


class WindJoystick(QtWidgets.QWidget):
    PLANE_CONFIGS = {
        "x-y": {
            "vertical_axis": "x",
            "horizontal_axis": "y",
        },
        "y-z": {
            "vertical_axis": "y",
            "horizontal_axis": "z",
        },
        "x-z": {
            "vertical_axis": "x",
            "horizontal_axis": "z",
        },
    }
    AXIS_COLORS = {
        "x": QtGui.QColor(220, 45, 45),
        "y": QtGui.QColor(40, 95, 220),
        "z": QtGui.QColor(40, 165, 95),
    }

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.pub = node.create_publisher(Vector3Stamped, "/crazyflie/in/wind", 10)

        self.max_force = 0.02
        self.deadzone_ratio = 0.12
        self.half_snap_ratio = 0.50
        self.half_snap_width = 0.06
        self.edge_snap_ratio = 0.92
        self.axis_snap_ratio = 0.18
        self.diagonal_snap_radius = 1.0 / math.sqrt(2.0)
        self.diagonal_snap_width = 0.12
        self.diagonal_snap_tolerance = 0.18
        self.handle = QtCore.QPointF(0.0, 0.0)
        self.dragging = False
        self.plane_name = "x-y"

        self.setWindowTitle("MuJoCo Wind Joystick")
        self.setMinimumSize(420, 520)

        self.max_force_spin = QtWidgets.QDoubleSpinBox()
        self.max_force_spin.setRange(0.0, 2.0)
        self.max_force_spin.setSingleStep(0.01)
        self.max_force_spin.setDecimals(3)
        self.max_force_spin.setValue(self.max_force)
        self.max_force_spin.setSuffix(" N")
        self.max_force_spin.valueChanged.connect(self.on_max_force_changed)

        self.plane_combo = QtWidgets.QComboBox()
        self.plane_combo.addItems(["x-y", "y-z", "x-z"])
        self.plane_combo.setCurrentText(self.plane_name)
        self.plane_combo.currentTextChanged.connect(self.on_plane_changed)

        self.force_label = QtWidgets.QLabel()
        self.force_label.setAlignment(QtCore.Qt.AlignCenter)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Max wind"))
        controls.addWidget(self.max_force_spin)
        controls.addSpacing(12)
        controls.addWidget(QtWidgets.QLabel("Plane"))
        controls.addWidget(self.plane_combo)

        layout = QtWidgets.QVBoxLayout()
        layout.addLayout(controls)
        layout.addStretch(1)
        layout.addWidget(self.force_label)
        self.setLayout(layout)

        self.publish_timer = QtCore.QTimer(self)
        self.publish_timer.timeout.connect(self.publish_wind)
        self.publish_timer.start(100)

        self.update_force_label()

    def on_max_force_changed(self, value):
        self.max_force = float(value)
        self.update_force_label()

    def on_plane_changed(self, value):
        if value in self.PLANE_CONFIGS:
            self.plane_name = value
            self.update_force_label()
            self.update()

    def current_plane_config(self):
        return self.PLANE_CONFIGS[self.plane_name]

    def joystick_rect(self):
        size = min(self.width() - 56, self.height() - 160)
        size = max(180, size)
        x = 0.5 * (self.width() - size)
        y = 72
        return QtCore.QRectF(x, y, size, size)

    def radius(self):
        return 0.5 * self.joystick_rect().width()

    def center(self):
        return self.joystick_rect().center()

    def point_to_handle(self, pos):
        c = self.center()
        r = self.radius()
        dx = pos.x() - c.x()
        dy = pos.y() - c.y()
        dist = math.hypot(dx, dy)
        if dist > r and dist > 1e-9:
            dx *= r / dist
            dy *= r / dist
        return QtCore.QPointF(dx / r, dy / r)

    def snap_to_axis(self, h):
        x = h.x()
        y = h.y()
        half_lo = self.half_snap_ratio - self.half_snap_width
        half_hi = self.half_snap_ratio + self.half_snap_width
        diag_lo = self.diagonal_snap_radius - self.diagonal_snap_width
        diag_hi = self.diagonal_snap_radius + self.diagonal_snap_width
        if half_lo <= abs(y) <= half_hi and abs(x) <= self.axis_snap_ratio:
            return QtCore.QPointF(0.0, math.copysign(self.half_snap_ratio, y))
        if half_lo <= abs(x) <= half_hi and abs(y) <= self.axis_snap_ratio:
            return QtCore.QPointF(math.copysign(self.half_snap_ratio, x), 0.0)
        if abs(y) >= self.edge_snap_ratio and abs(x) <= self.axis_snap_ratio:
            return QtCore.QPointF(0.0, math.copysign(1.0, y))
        if abs(x) >= self.edge_snap_ratio and abs(y) <= self.axis_snap_ratio:
            return QtCore.QPointF(math.copysign(1.0, x), 0.0)
        if diag_lo <= abs(x) <= diag_hi and diag_lo <= abs(y) <= diag_hi:
            if abs(abs(x) - abs(y)) <= self.diagonal_snap_tolerance:
                return QtCore.QPointF(
                    math.copysign(self.diagonal_snap_radius, x),
                    math.copysign(self.diagonal_snap_radius, y),
                )
        return h

    def set_handle_from_pos(self, pos):
        h = self.point_to_handle(pos)
        if math.hypot(h.x(), h.y()) < self.deadzone_ratio:
            h = QtCore.QPointF(0.0, 0.0)
        else:
            h = self.snap_to_axis(h)
        self.handle = h
        self.update_force_label()
        self.update()

    def wind_force(self):
        # Screen y is down. Up maps to the first selected axis, left to the second.
        cfg = self.current_plane_config()
        force_by_axis = {"x": 0.0, "y": 0.0, "z": 0.0}
        force_by_axis[cfg["vertical_axis"]] = -self.max_force * self.handle.y()
        force_by_axis[cfg["horizontal_axis"]] = -self.max_force * self.handle.x()
        return (
            force_by_axis["x"],
            force_by_axis["y"],
            force_by_axis["z"],
        )

    def update_force_label(self):
        fx, fy, fz = self.wind_force()
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        self.force_label.setText(
            f"plane {self.plane_name}  "
            f"wind force: x={fx:+.3f} N, y={fy:+.3f} N, z={fz:+.3f} N, |F|={mag:.3f} N"
        )

    def publish_wind(self):
        if not rclpy.ok():
            self.publish_timer.stop()
            return

        msg = Vector3Stamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        fx, fy, fz = self.wind_force()
        msg.vector.x = fx
        msg.vector.y = fy
        msg.vector.z = fz
        try:
            self.pub.publish(msg)
        except Exception:
            self.publish_timer.stop()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.joystick_rect().contains(event.pos()):
            self.dragging = True
            self.set_handle_from_pos(event.pos())

    def mouseMoveEvent(self, event):
        if self.dragging:
            self.set_handle_from_pos(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.dragging = False
            self.set_handle_from_pos(event.pos())

    def mouseDoubleClickEvent(self, event):
        if self.joystick_rect().contains(event.pos()):
            self.handle = QtCore.QPointF(0.0, 0.0)
            self.update_force_label()
            self.update()

    def paintEvent(self, event):
        super().paintEvent(event)

        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        rect = self.joystick_rect()
        c = rect.center()
        r = 0.5 * rect.width()
        dead_r = r * self.deadzone_ratio

        painter.setPen(QtGui.QPen(QtGui.QColor(45, 55, 65), 3))
        painter.setBrush(QtGui.QColor(245, 247, 250))
        painter.drawEllipse(c, r, r)

        painter.setPen(QtGui.QPen(QtGui.QColor(120, 130, 145), 1, QtCore.Qt.DashLine))
        painter.drawLine(QtCore.QPointF(c.x() - r, c.y()), QtCore.QPointF(c.x() + r, c.y()))
        painter.drawLine(QtCore.QPointF(c.x(), c.y() - r), QtCore.QPointF(c.x(), c.y() + r))

        painter.setPen(QtGui.QPen(QtGui.QColor(220, 80, 80), 2))
        painter.setBrush(QtGui.QColor(255, 230, 230, 180))
        painter.drawEllipse(c, dead_r, dead_r)

        self.draw_axis_snap_zones(painter, c, r)

        handle_pos = QtCore.QPointF(c.x() + self.handle.x() * r, c.y() + self.handle.y() * r)
        painter.setPen(QtGui.QPen(QtGui.QColor(20, 80, 160), 3))
        painter.drawLine(c, handle_pos)

        painter.setPen(QtGui.QPen(QtGui.QColor(15, 55, 120), 2))
        painter.setBrush(QtGui.QColor(65, 135, 230))
        painter.drawEllipse(handle_pos, 16, 16)

        painter.setPen(QtGui.QPen(QtGui.QColor(40, 45, 50), 1))
        painter.drawText(
            rect.adjusted(0, -32, 0, 0),
            QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop,
            f"drag to set {self.plane_name} wind; double-click to center",
        )

        self.draw_axis_hint(painter)

    def draw_axis_snap_zones(self, painter, center, radius):
        self.draw_half_snap_zones(painter, center, radius)
        self.draw_edge_snap_zones(painter, center, radius)
        self.draw_diagonal_snap_zones(painter, center, radius)

    def draw_half_snap_zones(self, painter, center, radius):
        zone_len = radius * self.axis_snap_ratio
        zone_depth = radius * (2.0 * self.half_snap_width)
        half_offset = radius * self.half_snap_ratio
        painter.setPen(QtGui.QPen(QtGui.QColor(70, 170, 120), 2))
        painter.setBrush(QtGui.QColor(215, 245, 225, 170))

        zones = [
            QtCore.QRectF(center.x() - zone_len, center.y() - half_offset - 0.5 * zone_depth,
                          2.0 * zone_len, zone_depth),
            QtCore.QRectF(center.x() - zone_len, center.y() + half_offset - 0.5 * zone_depth,
                          2.0 * zone_len, zone_depth),
            QtCore.QRectF(center.x() - half_offset - 0.5 * zone_depth, center.y() - zone_len,
                          zone_depth, 2.0 * zone_len),
            QtCore.QRectF(center.x() + half_offset - 0.5 * zone_depth, center.y() - zone_len,
                          zone_depth, 2.0 * zone_len),
        ]
        for zone in zones:
            painter.drawRoundedRect(zone, 3.0, 3.0)

    def draw_edge_snap_zones(self, painter, center, radius):
        zone_len = radius * self.axis_snap_ratio
        zone_depth = radius * (1.0 - self.edge_snap_ratio)
        painter.setPen(QtGui.QPen(QtGui.QColor(80, 145, 230), 2))
        painter.setBrush(QtGui.QColor(210, 230, 255, 170))

        zones = [
            QtCore.QRectF(center.x() - zone_len, center.y() - radius, 2.0 * zone_len, zone_depth),
            QtCore.QRectF(center.x() - zone_len, center.y() + radius - zone_depth, 2.0 * zone_len, zone_depth),
            QtCore.QRectF(center.x() - radius, center.y() - zone_len, zone_depth, 2.0 * zone_len),
            QtCore.QRectF(center.x() + radius - zone_depth, center.y() - zone_len, zone_depth, 2.0 * zone_len),
        ]
        for zone in zones:
            painter.drawRoundedRect(zone, 3.0, 3.0)

    def draw_diagonal_snap_zones(self, painter, center, radius):
        offset = radius * self.diagonal_snap_radius
        box_size = radius * (2.0 * self.diagonal_snap_width)
        half_box = 0.5 * box_size
        painter.setPen(QtGui.QPen(QtGui.QColor(235, 150, 50), 2))
        painter.setBrush(QtGui.QColor(255, 235, 200, 180))

        centers = [
            QtCore.QPointF(center.x() - offset, center.y() - offset),
            QtCore.QPointF(center.x() - offset, center.y() + offset),
            QtCore.QPointF(center.x() + offset, center.y() - offset),
            QtCore.QPointF(center.x() + offset, center.y() + offset),
        ]
        for zone_center in centers:
            zone = QtCore.QRectF(
                zone_center.x() - half_box,
                zone_center.y() - half_box,
                box_size,
                box_size,
            )
            painter.drawRoundedRect(zone, 4.0, 4.0)

    def draw_axis_hint(self, painter):
        base = QtCore.QPointF(36.0, self.height() - 36.0)
        x_tip = QtCore.QPointF(base.x(), base.y() - 56.0)
        y_tip = QtCore.QPointF(base.x() - 56.0, base.y())
        cfg = self.current_plane_config()
        vertical_axis = cfg["vertical_axis"]
        horizontal_axis = cfg["horizontal_axis"]

        self.draw_arrow(
            painter,
            base,
            x_tip,
            self.AXIS_COLORS[vertical_axis],
            f"+{vertical_axis}",
            QtCore.QPointF(8.0, -8.0),
        )
        self.draw_arrow(
            painter,
            base,
            y_tip,
            self.AXIS_COLORS[horizontal_axis],
            f"+{horizontal_axis}",
            QtCore.QPointF(-28.0, -8.0),
        )

    def draw_arrow(self, painter, start, end, color, label, label_offset):
        painter.setPen(QtGui.QPen(color, 3))
        painter.setBrush(color)
        painter.drawLine(start, end)

        angle = math.atan2(end.y() - start.y(), end.x() - start.x())
        head_len = 10.0
        head_angle = math.radians(28.0)
        p1 = QtCore.QPointF(
            end.x() - head_len * math.cos(angle - head_angle),
            end.y() - head_len * math.sin(angle - head_angle),
        )
        p2 = QtCore.QPointF(
            end.x() - head_len * math.cos(angle + head_angle),
            end.y() - head_len * math.sin(angle + head_angle),
        )
        painter.drawPolygon(QtGui.QPolygonF([end, p1, p2]))

        painter.setPen(QtGui.QPen(color, 2))
        painter.drawText(end + label_offset, label)


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--window-x", type=int, default=1480)
    parser.add_argument("--window-y", type=int, default=760)
    parser.add_argument("--window-width", type=int, default=420)
    parser.add_argument("--window-height", type=int, default=520)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node("wind_joystick")
    app = QtWidgets.QApplication([sys.argv[0]])
    widget = WindJoystick(node)
    widget.resize(args.window_width, args.window_height)
    widget.move(args.window_x, args.window_y)
    widget.show()
    try:
        exit_code = app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
