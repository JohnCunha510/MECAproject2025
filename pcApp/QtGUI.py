import sys
import serial
from serialCommThread import SerialThread
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QPushButton, QLineEdit, QHBoxLayout
from PyQt5 import QtGui
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import serial.tools.list_ports


def get_available_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

class PortComboBox(QComboBox):
    aboutToShowPopup = pyqtSignal()

    def showPopup(self):
        self.aboutToShowPopup.emit()  # Emit signal before showing
        super().showPopup()


# -------- Main Window with Plot --------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-time Serial Plot")
        self.setWindowIcon(QtGui.QIcon(r'MECAproject2025\plotingSerial\icon.png'))

        # Initialize data
        self.x_data = [0]
        self.x_data_voltage = list(range(1, 102))
        self.y_data_current = [0]
        self.y_data_voltage = [0] * 101
        self.y_data_speed = [0]
        self.y_data_torque = [0]
        self.y_data_error = [0]

        # Serial thread
        self.serial_thread = SerialThread(port='COM4', baudrate=9600)
        self.serial_thread.new_data.connect(self.receive_data)
        self.serial_thread.start()

        # Plot update timer
        self.timer = QTimer()
        self.timer.setInterval(100)  # Update every 100 ms
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

        # Matplotlib setup
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.current = self.figure.add_subplot(231)
        self.speed = self.figure.add_subplot(232)
        self.torque = self.figure.add_subplot(234)
        self.error = self.figure.add_subplot(235)
        self.voltage = self.figure.add_subplot(233)

        # ----- Left control panel -----
        self.input_field = QLineEdit()
        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_to_serial)
        self.port_selector = PortComboBox()
        self.port_selector.aboutToShowPopup.connect(self.refresh_ports)
        self.refresh_ports()
        #self.port_selector.addItems(["COM3", "COM4", "COM5"])  # Or dynamically detect ports
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.change_port)

        

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.input_field)
        left_layout.addWidget(self.send_button)
        left_layout.addStretch()  # Push controls to the top
        left_layout.addWidget(self.port_selector)
        left_layout.addWidget(self.connect_button)

        left_panel = QWidget()
        left_panel.setLayout(left_layout)


        # ----- Right plot panel -----
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.canvas)

        right_panel = QWidget()
        right_panel.setLayout(right_layout)

        # ----- Combine into main horizontal layout -----
        main_layout = QHBoxLayout()
        main_layout.addWidget(left_panel, 1)    # Stretch factor 1
        main_layout.addWidget(right_panel, 6)   # Stretch factor 4 (plot area is bigger)

        # Final container
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)
    # Function to refresh the port list
    def refresh_ports(self):
        self.port_selector.clear()
        ports = get_available_ports()
        self.port_selector.addItems(ports)
    
    def restart_serial_thread(self, new_port):
        if hasattr(self, 'serial_thread') and self.serial_thread.isRunning():
            self.serial_thread.stop()

        # Create new thread with new port
        self.serial_thread = SerialThread(port=new_port, baudrate=9600)
        self.serial_thread.new_data.connect(self.receive_data)
        self.serial_thread.start()

    # Send data to serial thread
    def send_to_serial(self):
        text = self.input_field.text()
        self.serial_thread.write_data(text)
    
    def change_port(self):
        selected_port = self.port_selector.currentText()
        self.restart_serial_thread(selected_port)

    def receive_data(self, value):
        self.y_data_current.append(value["current"])
        #value["command"] = 30
        self.y_data_voltage[1: value["command"]] = [9] * (value["command"]-1)
        self.y_data_voltage[value["command"]: -1] = [0] * (100 - value["command"])
        self.y_data_speed.append(value["speed"])
        self.y_data_torque.append(value["torque"])
        self.y_data_error.append(value["error"])
        print("[1] %d, [2] %d, [3] %d;" % (value["current"], value["command"], value["speed"]))
        self.x_data.append(self.x_data[-1] + 1)  # Simple x: count of values

        if len(self.y_data_current) > 100:
            self.x_data.pop(0)
            self.y_data_current.pop(0)
            #self.y_data_voltage.pop(0)
            self.y_data_speed.pop(0)
            self.y_data_torque.pop(0)
            self.y_data_error.pop(0)

    def update_plot(self):
        self.current.clear()
        self.current.plot(self.x_data, self.y_data_current)
        y_max = max(self.y_data_current)
        self.current.set_ylim(bottom=0, top=y_max * 1.2)  # 10% headroom
        self.current.set_title("Current")
        self.current.set_xlabel("Sample")
        self.current.set_ylabel("mA")

        self.voltage.clear()
        self.voltage.plot(self.x_data_voltage, self.y_data_voltage)
        y_max = max(self.y_data_voltage)
        self.voltage.set_ylim(bottom=0, top=y_max * 1.2)  # 10% headroom
        self.voltage.set_title("Voltage")
        self.voltage.set_xlabel("Sample")
        self.voltage.set_ylabel("mV")

        self.speed.clear()
        self.speed.plot(self.x_data, self.y_data_speed)
        y_max = max(self.y_data_speed)
        self.speed.set_ylim(bottom=0, top=y_max * 1.2)  # 10% headroom
        self.speed.set_title("speed")
        self.speed.set_xlabel("Sample")
        self.speed.set_ylabel("RPM")

        self.torque.clear()
        self.torque.plot(self.x_data, self.y_data_torque)
        y_max = max(self.y_data_torque)
        self.torque.set_ylim(bottom=0, top=y_max * 1.2)  # 10% headroom
        self.torque.set_title("torque")
        self.torque.set_xlabel("Sample")
        self.torque.set_ylabel("Nm")

        self.error.clear()
        self.error.plot(self.x_data, self.y_data_error)
        y_max = max(self.y_data_error)
        self.error.set_ylim(bottom=0, top=y_max * 1.2)  # 10% headroom
        self.error.set_title("error")
        self.error.set_xlabel("Sample")
        self.error.set_ylabel("")

        self.canvas.draw()

    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()