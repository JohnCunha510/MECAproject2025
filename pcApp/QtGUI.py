import sys
import serial
import math
from serialCommThread import SerialThread
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox, QSlider, QPushButton, QLabel
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt5.QtWidgets import QPushButton, QLineEdit, QHBoxLayout, QButtonGroup
from PyQt5 import QtGui
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import serial.tools.list_ports


# return available Serial Ports
def get_available_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# QComboBox with a way to detect when the list is opened
class PortComboBox(QComboBox):
    aboutToShowPopup = pyqtSignal()

    def showPopup(self):
        self.aboutToShowPopup.emit()  # Emit signal before showing
        super().showPopup()

# Control panel section and function
# -> MODE OPTION BUTTONS
# -> PID GAIN SLIDERS
# -> TORQUE SLIDER
# variables are sent, received and updated trhough the class functions
class ControlPanel(QWidget):
    def __init__(self, main_window_instance):
        self.main_window = main_window_instance
        super().__init__()
        self.control_mode = 0
        self.PID_Kp = 0
        self.PID_Ki = 0
        self.PID_Kd = 0
        self.set_torque = 0

        self.PID_Kp_reset = 0
        self.PID_Ki_reset = 0
        self.PID_Kd_reset = 0

        # create main layout
        layout = QVBoxLayout(self)

        # Title Lable Control Panel
        label = QLabel("Control Panel")
        label.setStyleSheet("font-size: 16pt; font-weight: bold;")
        layout.addWidget(label)

        # Control mode options (button arrray)
        layout.addWidget(QLabel("Operating Mode"))

        # Create layout for horizontal button row
        mode_layout = QHBoxLayout()

        # Create button group and buttons
        self.mode_group = QButtonGroup(self)
        self.mode_buttons = []

        modes = [("Speed", 1), ("Position", 2), ("Torque", 3)]
        for label, mode_id in modes:
            btn = QPushButton(label)
            btn.setCheckable(True)
            self.mode_group.addButton(btn, mode_id)
            mode_layout.addWidget(btn)
            self.mode_buttons.append(btn)

        # Add the horizontal button layout
        layout.addLayout(mode_layout)

        # Set default mode
        self.mode_buttons[0].setChecked(True)
        self.control_mode = 1  # default to Speed

        # Connect signal
        self.mode_group.buttonClicked[int].connect(self.set_mode)

        # Create layout for PID values
        PID_layout = QHBoxLayout()

        # left layout for Lables
        PID_layout_left = QVBoxLayout()

        PID_name_P = QLabel("P")
        PID_name_P.setStyleSheet("font-size: 16pt; font-weight: bold;")
        PID_name_P.setAlignment(Qt.AlignCenter)
        PID_layout_left.addWidget(PID_name_P)

        PID_name_I = QLabel("I")
        PID_name_I.setStyleSheet("font-size: 16pt; font-weight: bold;")
        PID_name_I.setAlignment(Qt.AlignCenter)
        PID_layout_left.addWidget(PID_name_I)

        PID_name_D = QLabel("D")
        PID_name_D.setStyleSheet("font-size: 16pt; font-weight: bold;")
        PID_name_D.setAlignment(Qt.AlignCenter)
        PID_layout_left.addWidget(PID_name_D)

        # right layout for PID sliders
        PID_layout_right = QVBoxLayout()

        self.slider_Kp = QSlider(Qt.Horizontal)
        self.slider_Kp.setRange(0, 10)
        self.slider_Kp.setValue(1)
        self.slider_Kp.setTickPosition(QSlider.TicksBelow)
        self.slider_Kp.setTickInterval(1)
        self.slider_Kp.valueChanged.connect(self.update_PID_Kp)
        PID_layout_right.addWidget(self.slider_Kp)

        self.slider_Ki = QSlider(Qt.Horizontal)
        self.slider_Ki.setRange(0, 10)
        self.slider_Ki.setValue(1)
        self.slider_Ki.setTickPosition(QSlider.TicksBelow)
        self.slider_Ki.setTickInterval(1)
        self.slider_Ki.valueChanged.connect(self.update_PID_Ki)
        PID_layout_right.addWidget(self.slider_Ki)

        self.slider_Kd = QSlider(Qt.Horizontal)
        self.slider_Kd.setRange(0, 10)
        self.slider_Kd.setValue(1)
        self.slider_Kd.setTickPosition(QSlider.TicksBelow)
        self.slider_Kd.setTickInterval(1)
        self.slider_Kd.valueChanged.connect(self.update_PID_Kd)
        PID_layout_right.addWidget(self.slider_Kd)

        PID_layout.addLayout(PID_layout_left, 1)
        PID_layout.addLayout(PID_layout_right, 6)
        
        layout.addLayout(PID_layout)

        # --- RESET BUTTON ---
        reset_button = QPushButton("Reset PID")
        reset_button.clicked.connect(self.reset_sliders)
        layout.addWidget(reset_button)

        # create layout for torque slider
        torque_layout = QHBoxLayout()

        self.slider_torque = QSlider(Qt.Horizontal)
        self.slider_torque.setRange(0, 100)
        self.slider_torque.setValue(50)
        self.slider_torque.setTickPosition(QSlider.TicksBelow)
        self.slider_torque.setTickInterval(30)
        self.slider_torque.valueChanged.connect(self.update_torque_slider)
        torque_layout.addWidget(QLabel("Set Torque"))
        torque_layout.addWidget(self.slider_torque)

        layout.addLayout(torque_layout)

    # Reset sliders button function
    def reset_sliders(self):
        self.slider_Kp.setValue(self.PID_Kp_reset)
        self.slider_Ki.setValue(self.PID_Ki_reset)
        self.slider_Kd.setValue(self.PID_Kd_reset)

    # Control mode change function
    def set_mode(self, mode_id):
        self.control_mode = mode_id
        if mode_id == 1:
            self.PID_Kp_reset = 0
            self.PID_Ki_reset = 0
            self.PID_Kd_reset = 0
        elif mode_id == 2:
            self.PID_Kp_reset = 0
            self.PID_Ki_reset = 0
            self.PID_Kd_reset = 0
        elif mode_id == 3:
            self.PID_Kp_reset = 0
            self.PID_Ki_reset = 0
            self.PID_Kd_reset = 0
        self.main_window.send_to_serial(0x03, mode_id)

    # update PID Kp parameter from the slider
    def update_PID_Kp(self, val):
        self.PID_Kp = self.slider_Kp.value()
        self.main_window.send_to_serial(0x04, self.PID_Kp*100)
    
    def update_PID_Ki(self, val):
        self.PID_Ki = self.slider_Ki.value()
        self.main_window.send_to_serial(0x05, self.PID_Ki*100)

    def update_PID_Kd(self, val):
        self.PID_Kd = self.slider_Kd.value()
        self.main_window.send_to_serial(0x06, self.PID_Kd*100)
    
    # update set_torque variable from the slider
    def update_torque_slider(self, val):
        self.set_torque = val
        self.main_window.send_to_serial(0x07, val)



# Motor visualisation class
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
        if color != Qt.red:
            needle_length = radius*0.7
        else:
            needle_length = radius
        painter.setPen(QPen(color, width))
        angle_rad = math.radians(-angle_deg + 90)
        x1 = center.x() + (radius - needle_length) * math.cos(angle_rad)
        y1 = center.y() - (radius - needle_length) * math.sin(angle_rad)
        x2 = center.x() + radius * math.cos(angle_rad)
        y2 = center.y() - radius * math.sin(angle_rad)
        painter.drawLine(int(x1), int(y1), int(x2), int(y2))

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
        self.draw_needle(painter, center, radius, self.actual_angle, Qt.red, 10)
        self.draw_needle(painter, center, radius, self.target_angle, Qt.blue, 8)

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

    def set_target_angle(self, angle):
        self.target_angle = angle % 360
        self.update()

    def set_actual_angle(self, angle):
        self.actual_angle = angle % 360
        self.update()

# -------- Main Window with Plot --------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-time Serial Plot")
        self.setWindowIcon(QtGui.QIcon(r'MECAproject2025\plotingSerial\icon.png'))
        self.setGeometry(100, 100, 1200, 800)

        # Initialize data
        self.x_data = [0]
        self.x_data_voltage = list(range(1, 102))
        self.y_data_current = [0]
        self.y_data_voltage = [0] * 101
        self.y_data_speed = [0]
        self.y_data_torque = [0]
        self.y_data_error = [0]
        self.y_data_other = [0]
        self.actual_angle = 0
        self.target_angle = 90

        # Serial thread
        self.serial_thread = SerialThread(port='COM19', baudrate=9600)
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
        self.other = self.figure.add_subplot(236)

        # ----- Left control panel -----
        window = self
        self.control_display = ControlPanel(window)
        self.control_display.setMinimumWidth(300)
        self.control_display.setMaximumHeight(500)

        self.motor_display = MotorWidget()
        self.motor_display.setMinimumWidth(300)
        self.motor_display.setMaximumHeight(500)
        

        self.port_selector = PortComboBox()
        self.port_selector.aboutToShowPopup.connect(self.refresh_ports)
        self.refresh_ports()
        #self.port_selector.addItems(["COM3", "COM4", "COM5"])  # Or dynamically detect ports
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.change_port)
        

        

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.control_display)
        left_layout.addStretch()  # Push controls to the top
        left_layout.addWidget(self.motor_display, 1)
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
    def send_to_serial(self, command_id, value):
        self.serial_thread.write_data(command_id, 1, value)
    
    def change_port(self):
        selected_port = self.port_selector.currentText()
        self.restart_serial_thread(selected_port)

    def receive_data(self, value):
        self.y_data_current.append(value["current"])
        #value["command"] = 30
        value["command"] = max(0, min(100, value["command"]))
        self.y_data_voltage[1: value["command"]] = [9] * (value["command"]-1)
        self.y_data_voltage[value["command"]: -1] = [0] * (100 - value["command"])
        self.y_data_speed.append(value["speed"])
        self.y_data_torque.append(value["torque"])
        self.y_data_error.append(value["error"])
        self.y_data_other.append(value["other"])

        #print("[1] %d, [2] %d, [3] %d;" % (value["current"], value["command"], value["speed"]))
        self.x_data.append(self.x_data[-1] + 1)  # Simple x: count of values

        if len(self.y_data_current) > 100:
            self.x_data.pop(0)
            self.y_data_current.pop(0)
            #self.y_data_voltage.pop(0)
            self.y_data_speed.pop(0)
            self.y_data_torque.pop(0)
            self.y_data_error.pop(0)
            self.y_data_other.pop(0)

    def update_plot(self):
        self.current.clear()
        self.current.plot(self.x_data, self.y_data_current)
        y_max = max(self.y_data_current)
        self.current.set_ylim(bottom=0, top=1+y_max * 1.2)  # 10% headroom
        self.current.set_title("Current")
        self.current.set_xlabel("Sample")
        self.current.set_ylabel("mA")

        self.voltage.clear()
        self.voltage.plot(self.x_data_voltage, self.y_data_voltage)
        y_max = max(self.y_data_voltage)
        self.voltage.set_ylim(bottom=0, top=0.1+y_max * 1.2)  # 10% headroom
        self.voltage.set_title("Voltage")
        self.voltage.set_xlabel("Sample")
        self.voltage.set_ylabel("mV")

        self.speed.clear()
        self.speed.plot(self.x_data, self.y_data_speed)
        y_max = max(self.y_data_speed)
        self.speed.set_ylim(bottom=0, top=1+y_max * 1.2)  # 10% headroom
        self.speed.set_title("speed")
        self.speed.set_xlabel("Sample")
        self.speed.set_ylabel("RPM")

        self.torque.clear()
        self.torque.plot(self.x_data, self.y_data_torque)
        y_max = max(self.y_data_torque)
        self.torque.set_ylim(bottom=0, top=1+y_max * 1.2)  # 10% headroom
        self.torque.set_title("torque")
        self.torque.set_xlabel("Sample")
        self.torque.set_ylabel("Nm")

        self.error.clear()
        self.error.plot(self.x_data, self.y_data_error)
        y_max = max(self.y_data_error)
        self.error.set_ylim(bottom=0, top=1+y_max * 1.2)  # 10% headroom
        self.error.set_title("error")
        self.error.set_xlabel("Sample")
        self.error.set_ylabel("")

        self.other.clear()
        self.other.plot(self.x_data, self.y_data_other)
        y_max = max(self.y_data_other)
        self.other.set_ylim(bottom=0, top=1+y_max * 1.2)  # 10% headroom
        self.other.set_title("other")
        self.other.set_xlabel("Sample")
        self.other.set_ylabel("")


        self.figure.tight_layout()
        self.canvas.draw()


    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()