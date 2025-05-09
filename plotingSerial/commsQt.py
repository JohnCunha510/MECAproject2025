import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QPushButton, QLineEdit, QHBoxLayout
from PyQt5 import QtGui
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import serial.tools.list_ports

# -------- Serial Thread --------
class SerialThread(QThread):
    new_data = pyqtSignal(int)

    def __init__(self, port='COM4', baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None

    def run(self):
        ser = serial.Serial(self.port, self.baudrate)
        while True:
            try:
                line = ser.read() #.decode('utf-8')
                value = int.from_bytes(line) #, byteorder='big', signed=True)  # adjust this if your data format is different
                self.new_data.emit(value)
            except Exception as e:
                print("Serial error:", e)
    
    def write_data(self, data):
        if self.ser and self.ser.is_open:
            self.ser.write((data + '\n').encode('utf-8'))

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.quit()
        self.wait()

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
        self.y_data = [0]

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
        self.current = self.figure.add_subplot(111)

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
        main_layout.addWidget(right_panel, 4)   # Stretch factor 4 (plot area is bigger)

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
        self.y_data.append(value)
        print("[1] %d" % (value))
        self.x_data.append(self.x_data[-1] + 1)  # Simple x: count of values
        if len(self.y_data) > 100:
            self.x_data.pop(0)
            self.y_data.pop(0)

    def update_plot(self):
        self.current.clear()
        self.current.plot(self.x_data, self.y_data)
        self.current.set_title("Current")
        self.current.set_xlabel("Sample")
        self.current.set_ylabel("mA")
        self.canvas.draw()

    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()

# -------- Run App --------
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

