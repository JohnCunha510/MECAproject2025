from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QTextEdit,
from PyQt5.QtCore import QTimer
from PyQt5 import QtGui
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import random

class TelemetryWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rocket Telemetry")
        self.setWindowIcon(QtGui.QIcon('icon.png'))

        self.x_data = []
        self.y_data = []

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.fig = Figure()
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.layout.addWidget(self.canvas)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.layout.addWidget(self.log)

        self.button = QPushButton("Send Command")
        self.button.clicked.connect(self.send_command)
        self.layout.addWidget(self.button)

        self.timer = QTimer()
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.read_serial_sim)
        self.timer.start()

    def send_command(self):
        self.log.append("Sent: CMD")

    def read_serial_sim(self):
        # Simulate telemetry value
        val = random.uniform(0, 100)
        self.log.append(f"Received: {val:.2f}")

        self.y_data.append(val)
        self.x_data.append(len(self.x_data))

        self.plot()

    def plot(self):
        self.ax.clear()
        self.ax.plot(self.x_data[-50:], self.y_data[-50:], 'r-')
        self.ax.set_ylabel("Altitude (m)")
        self.ax.set_xlabel("Time (s)")
        self.canvas.draw()
