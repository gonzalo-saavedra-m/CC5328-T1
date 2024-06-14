from ui_embebidos import Ui_Dialog

class UI_Controller():
    def __init__(self, parent):
        self.ui = Ui_Dialog()
        self.parent = parent

    def setSignals(self):
        self.ui.start_button.clicked.connect(self.start)

    def start(self):
        print('Starting...')
        selected_sensor = self.ui.sensor_selector.currentText()
        print(f'Selected sensor: {selected_sensor}')
        if selected_sensor == 'BMI270':
            power_mode = self.ui.bmi270_powermode.currentIndex()
            print(f'Power mode: {power_mode}')
            acc_odr = self.ui.bmi270_acc_odr.currentIndex()
            print(f'Accelerometer ODR: {acc_odr}')
            acc_sens = self.ui.bmi270_acc_sens.currentIndex()
            print(f'Accelerometer sensitivity: {acc_sens}')
            gyro_odr = self.ui.bmi270_gyro_odr.currentIndex()
            print(f'Gyroscope ODR: {gyro_odr}')
            gyro_sens = self.ui.bmi270_gyro_sens.currentIndex()
            print(f'Gyroscope sensitivity: {gyro_sens}')
        elif selected_sensor == 'BME688':
            power_mode = self.ui.bme688_powermode.currentIndex()
            print(f'Power mode: {power_mode}')


if __name__ == "__main__":
    from PyQt5 import QtWidgets
    import sys
    app = app = QtWidgets.QApplication(sys.argv)
    dialog = QtWidgets.QDialog()
    ui_controller = UI_Controller(parent=dialog)
    ui = ui_controller.ui
    ui.setupUi(dialog)
    dialog.show()
    ui_controller.setSignals()
    sys.exit(app.exec_())