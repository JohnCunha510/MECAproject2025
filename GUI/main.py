from PyQt5.QtWidgets import QApplication
from telemetry_gui import TelemetryWindow
import sys

app = QApplication(sys.argv)
window = TelemetryWindow()
window.show()
sys.exit(app.exec_())
