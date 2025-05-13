import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel
)
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
from PyQt5.QtCore import Qt, QTimer
import math


class MotorWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.actual_angle = 0
        self.target_angle = 90

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_angles)
        self.timer.start(100)

    def update_angles(self):
        if self.actual_angle != self.target_angle:
            diff = (self.target_angle - self.actual_angle) % 360
            if diff > 180:
                self.actual_angle -= 5
            else:
                self.actual_angle += 5
            self.actual_angle %= 360
        self.update()

    def draw_needle(self, painter, center, radius, angle_deg, color, width):
        painter.setPen(QPen(color, width))
        angle_rad = math.radians(-angle_deg + 90)
        x = center.x() + radius * math.cos(angle_rad)
        y = center.y() - radius * math.sin(angle_rad)
        painter.drawLine(center.x(), center.y(), int(x), int(y))

    def draw_ticks(self, painter, center, radius):
        tick_length = 10
        painter.setPen(QPen(Qt.black, 2))
        for angle in range(0, 360, 30):
            angle_rad = math.radians(-angle + 90)
            x1 = center.x() + (radius - tick_length) * math.cos(angle_rad)
            y1 = center.y() - (radius - tick_length) * math.sin(angle_rad)
            x2 = center.x() + radius * math.cos(angle_rad)
            y2 = center.y() - radius * math.sin(angle_rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

    def paintEvent(self, event):
        center = self.rect().center()
        radius = min(self.width(), self.height()) // 2 - 10

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.setBrush(QBrush(Qt.lightGray))
        painter.setPen(QPen(Qt.black, 2))
        painter.drawEllipse(center, radius, radius)

        self.draw_ticks(painter, center, radius)
        self.draw_needle(painter, center, radius, self.actual_angle, Qt.red, 5)
        self.draw_needle(painter, center, radius, self.target_angle, Qt.blue, 4)

        # Draw angle info
        painter.setPen(Qt.black)
        painter.setFont(QFont("Arial", 12))
        painter.drawText(10, 20, f"Target: {int(self.target_angle)}°")
        painter.drawText(10, 40, f"Actual: {int(self.actual_angle)}°")

    def mousePressEvent(self, event):
        center = self.rect().center()
        dx = event.x() - center.x()
        dy = center.y() - event.y()
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        angle = (90 - angle_deg) % 360
        self.target_angle = angle
        self.update()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Display App - Two Columns")
        self.setGeometry(100, 100, 600, 300)

        layout = QHBoxLayout(self)

        # Left: Motor display
        self.motor_display = MotorWidget()
        self.motor_display.setMinimumWidth(300)
        layout.addWidget(self.motor_display, 1)

        # Right: Control panel placeholder
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.addWidget(QLabel("Controls will go here"))
        right_layout.addStretch()
        layout.addWidget(right_panel, 1)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
