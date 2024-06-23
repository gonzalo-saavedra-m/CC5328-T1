from controllers.ui_embebidos import Ui_Dialog
import pyqtgraph as pg
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.ticker as ticker
import numpy as np
import queue

from PyQt5 import QtCore, QtWidgets, QtGui, uic
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QVBoxLayout

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class UI_Controller():
    def __init__(self, parent=None):
        self.parent = parent
        self.ui = Ui_Dialog()
        self.ui.setupUi(self.parent)
        self.ui.start_button.clicked.connect(self.start)
        self.start_callback = lambda **kwargs: None
        self.canvases = {}
        self.labels = {}
        self.reference_plot = None

        # Create canvas for each widget
        for i in range(1, 13):
            widget = getattr(self.ui, f'widget_{i}')
            label = getattr(self.ui, f'datalabel_{i}')
            canvas = MplCanvas(self, width=5, height=4, dpi=100)
            layout = QVBoxLayout()
            layout.addWidget(canvas)
            widget.setLayout(layout)
            self.canvases[f'widget_{i}'] = canvas
            self.labels[f'datalabel_{i}'] = label

    def set_start_callback(self, callback: callable):
        self.start_callback = callback

    def start(self):
        kwargs = {}
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

    def show_bmi270_data(self,
                        #  acc: dict, gyr: dict, RMS: dict, FFT: dict, peaks: dict
                         ):
        """plots the data from the BMI270 sensor.
        acc: { x: [], y: [], z: [] },
        gyr: { x: [], y: [], z: [] },
        RMS: { acc_x: [], acc_y: [], acc_z: [] },
        FFT: { acc_x: { r: [], i: [] }, acc_y: { r: [], i: [] }, acc_z: { r: [], i: [] } },
        peaks: { acc_x: [], acc_y: [], acc_z: [], gyr_x: [], gyr_y: [], gyr_z: [], RMS.acc_x: [], RMS.acc_y: [], RMS.acc_z: [] }
        """
        time = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        widget_number = 1
        canvas: MplCanvas = self.canvases.get(f'widget_{widget_number}')
        label = self.labels.get(f'datalabel_{widget_number}')
        if canvas and label:
            canvas.axes.clear()
            canvas.axes.plot(time)
            canvas.draw()
            label.setText('Sample data')
