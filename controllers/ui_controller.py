from controllers.ui_embebidos import Ui_Dialog

class UI_Controller():
    def __init__(self, parent):
        self.parent = parent
        self.ui = Ui_Dialog()
        self.ui.setupUi(self.parent)
        self.ui.start_button.clicked.connect(self.start)
        self.start_callback = lambda **kwargs: None

    def set_start_callback(self, callback: callable):
        self.start_callback = callback

    def start(self):
        kwargs = {}
        print('Starting...')
        selected_sensor = self.ui.sensor_selector.currentText()
        kwargs['selected_sensor'] = selected_sensor
        if selected_sensor == 'BMI270':
            kwargs['powermode'] = self.ui.bmi270_powermode.currentText()
            kwargs['acc_odr'] = int(self.ui.bmi270_acc_odr.currentText())
            kwargs['acc_sens'] = int(self.ui.bmi270_acc_sens.currentText())
            kwargs['gyro_odr'] = int(self.ui.bmi270_gyro_odr.currentText())
            kwargs['gyro_sens'] = int(self.ui.bmi270_gyro_sens.currentText())
        elif selected_sensor == 'BME688':
            kwargs['powermode'] = self.ui.bme688_powermode.currentText()
        self.start_callback(**kwargs)


if __name__ == "__main__":
    from PyQt5 import QtWidgets
    import sys
    app = app = QtWidgets.QApplication(sys.argv)
    dialog = QtWidgets.QDialog()
    ui_controller = UI_Controller(parent=dialog)
    dialog.show()
    ui_controller.set_signals()
    sys.exit(app.exec_())