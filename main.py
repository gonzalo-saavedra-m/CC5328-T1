import sys
from controllers import *
from PyQt5 import QtWidgets

PORT = 'COM3'
BMI270_BAUD_RATE = 115200
BME688_BAUD_RATE = 115200
DATA_SIZE = 100

bmi270_receiver = BMI270_Receiver(PORT, BMI270_BAUD_RATE, DATA_SIZE)
bme688_receiver = BME688_Receiver(PORT, BME688_BAUD_RATE, DATA_SIZE)

app = app = QtWidgets.QApplication(sys.argv)
dialog = QtWidgets.QDialog()
ui_controller = UI_Controller(parent=dialog)
ui = ui_controller.ui
ui.setupUi(dialog)
dialog.show()
ui_controller.setSignals()
ui_controller = UI_Controller()
