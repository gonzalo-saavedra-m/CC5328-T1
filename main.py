import sys
from controllers import *
from PyQt5 import QtWidgets
import serial
from pprint import pprint

PORT = 'COM3'
BAUD_RATE = 115200
DATA_SIZE = 100

ser = serial.Serial(PORT, BAUD_RATE, timeout=5)
bmi270_receiver = BMI270_Receiver(ser, DATA_SIZE)
bme688_receiver = BME688_Receiver(ser)
app = QtWidgets.QApplication(sys.argv)
dialog = QtWidgets.QDialog()
ui_controller = UI_Controller(parent=dialog)

def start_callback(selected_sensor: str, powermode: str, **kwargs):
    if selected_sensor == 'BMI270':
        data = bmi270_receiver.read(powermode, **kwargs)
        ui_controller.show_bmi270_data(**data)
    elif selected_sensor == 'BME688':
        data = bme688_receiver.read(powermode)
        if data is None:
            ui_controller.clear_plots()
        else:
            ui_controller.show_bme688_data(**data)

ui_controller.set_start_callback(start_callback)
dialog.show()
sys.exit(app.exec_())