#!/usr/bin/env python3

import math
import sys
import argparse

import rclpy
from geometry_msgs.msg import Vector3Stamped
from PyQt5 import QtCore, QtGui, QtWidgets


class WindJoystick(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.pub = node.create_publisher(Vector3Stamped, "/crazyflie/in/wind", 10)

        self.max_force = 0.08
        self.deadzone_ratio = 0.12
        self.handle = QtCore.QPointF(0.0, 0.0)
        self.dragging = False

        self.setWindowTitle("MuJoCo Wind Joystick")
        self.setMinimumSize(420, 520)

        self.max_force_spin = QtWidgets.QDoubleSpinBox()
        self.max_force_spin.setRange(0.0, 2.0)
        self.max_force_spin.setSingleStep(0.01)
        self.max_force_spin.setDecimals(3)
        self.max_force_spin.setValue(self.max_force)
        self.max_force_spin.setSuffix(" N")
        self.max_force_spin.valueChanged.connect(self.on_max_force_changed)

        self.force_label = QtWidgets.QLabel()
        self.force_label.setAlignment(QtCore.Qt.AlignCenter)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Max wind"))
        controls.addWidget(self.max_force_spin)

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

    def set_handle_from_pos(self, pos):
        h = self.point_to_handle(pos)
        if math.hypot(h.x(), h.y()) < self.deadzone_ratio:
            h = QtCore.QPointF(0.0, 0.0)
        self.handle = h
        self.update_force_label()
        self.update()

    def wind_force(self):
        # Screen y is down. Up maps to +world-x, left maps to +world-y.
        return (
            -self.max_force * self.handle.y(),
            -self.max_force * self.handle.x(),
            0.0,
        )

    def update_force_label(self):
        fx, fy, fz = self.wind_force()
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        self.force_label.setText(
            f"wind force: x={fx:+.3f} N, y={fy:+.3f} N, |F|={mag:.3f} N"
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
            "drag and release to hold wind; double-click to center",
        )

        self.draw_axis_hint(painter)

    def draw_axis_hint(self, painter):
        base = QtCore.QPointF(36.0, self.height() - 36.0)
        x_tip = QtCore.QPointF(base.x(), base.y() - 56.0)
        y_tip = QtCore.QPointF(base.x() - 56.0, base.y())

        self.draw_arrow(
            painter,
            base,
            x_tip,
            QtGui.QColor(220, 45, 45),
            "+x",
            QtCore.QPointF(8.0, -8.0),
        )
        self.draw_arrow(
            painter,
            base,
            y_tip,
            QtGui.QColor(40, 95, 220),
            "+y",
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
