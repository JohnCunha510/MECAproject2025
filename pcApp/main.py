from PyQt5.QtWidgets import QApplication
from QtGUI import MainWindow
import sys

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
