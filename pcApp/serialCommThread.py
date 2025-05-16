import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QPushButton, QLineEdit, QHBoxLayout
from PyQt5 import QtGui
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import serial.tools.list_ports

FRAME_START = 0x8A
FRAME_ESCAPE_CHAR = 0x8B
FRAME_XOR_CHAR = 0x20

RCV_ST_IDLE = 0
RCV_ST_CMD = 1
RCV_ST_DATA_LENGTH = 2
RCV_ST_DATA = 3
RCV_ST_CHECKSUM = 4

DATA_COMMAND = ["position", "current", "speed", "torque", "error", "command", "other", "all"]




# -------- Serial Thread --------
class SerialThread(QThread):
    data_frame = {"position": 0, "current": 0, "speed": 0, "torque": 0, "error": 0, "command": 0, "other": 0}
    new_data = pyqtSignal(object)

    def __init__(self, port='COM19', baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None
        self.receiverStatus = RCV_ST_IDLE
        self.in_byte = 0x00
        self.value = 0
        self.xored = 0x00
        self.in_frame = []
        self.out_frame = []
        self.checksum = 0x00
        self.dataLength = 0
        self.n_byte = 0

        self.data_name = ""


    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            while self.running:
                self.in_byte = self.ser.read(1) #.decode('utf-8')
                self.value = int.from_bytes(self.in_byte, byteorder='big', signed=False)

                if self.in_byte == FRAME_ESCAPE_CHAR:
                    self.xored = FRAME_XOR_CHAR
                else:
                    self.value ^= self.xored
                    self.xored = 0x00
                    self.in_byte = self.value
                    

                    if self.receiverStatus == RCV_ST_IDLE:
                        #print("[10] %3X;" % (self.value))
                        if self.in_byte == FRAME_START:
                            #print("[20] %3X;" % (self.value))
                            self.in_frame.clear()
                            self.in_frame.append(self.value)
                            self.checksum = self.in_byte
                            self.receiverStatus = RCV_ST_CMD

                    elif self.receiverStatus == RCV_ST_CMD:
                        print("[30] %3X;" % (self.value))
                        self.in_frame.append(self.value)
                        self.checksum += self.in_byte
                        if self.value >= 10 & self.value <=17:
                            self.data_name = DATA_COMMAND[self.value -10]
                            #self.data_frame[self.data_name] = 0
                        self.receiverStatus = RCV_ST_DATA_LENGTH

                    elif self.receiverStatus == RCV_ST_DATA_LENGTH:
                        #print("[40] %3X;" % (self.value))
                        self.dataLength = self.value
                        self.n_byte = self.value
                        self.in_frame.append(self.value)
                        self.checksum += self.in_byte
                        self.receiverStatus = RCV_ST_DATA

                    elif self.receiverStatus == RCV_ST_DATA:
                        print("[50] %3d;" % (self.value))
                        self.in_frame.append(self.value)
                        if self.n_byte == self.dataLength:
                            self.data_frame[self.data_name] = self.value << (0 + (self.n_byte-1)*8)
                        else:
                            self.data_frame[self.data_name] |= self.value << (0 + (self.n_byte-1)*8)
                        self.checksum += self.in_byte
                        self.n_byte -= 1
                        if self.n_byte == 0:
                            self.receiverStatus = RCV_ST_CHECKSUM
                        elif self.n_byte < 0:
                            self.receiverStatus = RCV_ST_IDLE
                        #print("[51] %3X;" % (self.n_byte))

                    elif self.receiverStatus == RCV_ST_CHECKSUM:
                        #print("[60] %3X;" % (self.value))
                        #print("[61] %3X;" % (self.checksum & 0xFF))
                        if self.in_byte == (self.checksum & 0xFF):
                            self.in_frame.append(self.value)
                            #print("[62] %3X, %d;" % (self.data_frame["current"], self.data_frame["current"]))
                            #print("[63] %3X, %d;" % (self.data_frame["command"], self.data_frame["command"]))
                            #print("[64] %3X, %d;" % (self.data_frame["speed"], self.data_frame["speed"]))
                            self.new_data.emit(self.data_frame)

                        self.receiverStatus = RCV_ST_IDLE

        except Exception as e:
            print("Serial error:", e)
    
    def write_data(self, command_id, n_data, value):
        self.out_frame.clear()
        self.out_frame.append(FRAME_START)
        self.out_frame.append(command_id)
        if command_id == 0x03:
            self.out_frame.append(n_data)
            self.out_frame.append(value & 0x00FF)
        else:
            self.out_frame.append(n_data * 2)
            self.out_frame.append((value >> 8) & 0x00FF)
            self.out_frame.append(value & 0x00FF)

        self.out_frame.append(self.calculate_checksum() & 0x00FF)

        #print(self.out_frame)
        
        if self.ser and self.ser.is_open:
            self.ser.write(bytes(self.out_frame))

    def calculate_checksum(self):
        checksum = 0x00
        for byte in self.out_frame:
            checksum += byte
            #print("[101] %3X;" % (byte))
        #print("[102] %3X;" % (checksum))
        return checksum

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.quit()
        self.wait()