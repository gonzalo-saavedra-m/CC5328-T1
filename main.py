import sys
from controllers import *
from PyQt5 import QtWidgets
import serial

PORT = 'COM3'
BAUD_RATE = 115200
DATA_SIZE = 100

ser = serial.Serial(PORT, BAUD_RATE, timeout=5)
bmi270_receiver = BMI270_Receiver(ser, DATA_SIZE)
bme688_receiver = BME688_Receiver(ser, DATA_SIZE)

def start_callback(selected_sensor: str, powermode: str, **kwargs):
    if selected_sensor == 'BMI270':
        print(kwargs)
    elif selected_sensor == 'BME688':
        data = bme688_receiver.read(powermode)
        # TODO: plot data

# Main
app = QtWidgets.QApplication(sys.argv)
dialog = QtWidgets.QDialog()
ui_controller = UI_Controller(parent=dialog)
ui_controller.set_start_callback(start_callback)
dialog.show()
sys.exit(app.exec_())
