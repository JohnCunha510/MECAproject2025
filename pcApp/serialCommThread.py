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
    new_data = pyqtSignal(float)

    def __init__(self, port='COM4', baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate)
            while self.running:
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