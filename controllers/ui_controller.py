from controllers.ui_embebidos import Ui_Dialog
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
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
        for i in range(1, 16):
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
        self.clear_plots()
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
                         acc: dict, gyr: dict, RMS: dict, FFT: dict, peaks: dict
                         ):
        """plots the data from the BMI270 sensor.
        acc: { x: [], y: [], z: [] },
        gyr: { x: [], y: [], z: [] },
        RMS: { acc_x: [], acc_y: [], acc_z: [] },
        FFT: { acc_x: { r: [], i: [] }, acc_y: { r: [], i: [] }, acc_z: { r: [], i: [] } },
        peaks: { acc_x: [], acc_y: [], acc_z: [], gyr_x: [], gyr_y: [], gyr_z: [], RMS.acc_x: [], RMS.acc_y: [], RMS.acc_z: [] }
        """
        for i, coord in enumerate(['x', 'y', 'z']):
            self.set_line_plot(acc[coord], i*2 + 1, f'Accelerometer {coord.upper()}')
            self.set_line_plot(gyr[coord], i*2 + 2, f'Gyroscope {coord.upper()}')
            self.set_line_plot(RMS[f'acc_{coord}'], i + 7, f'RMS Accelerometer {coord.upper()}')
            self.set_2d_plot(FFT[f'acc_{coord}']['r'], FFT[f'acc_{coord}']['i'], i + 10, f'FFT Accelerometer {coord.upper()}')
            self.set_scatter_plot(peaks[f'acc_{coord}'], i + 13, f'Peaks Accelerometer {coord.upper()}')

    def show_bme688_data(self, **kwargs):
        # TODO: Implement this method
        # raise NotImplementedError
        pass


    def set_2d_plot(self, x: list, y: list, widget_number: int, title: str):
        canvas: MplCanvas = self.canvases.get(f'widget_{widget_number}')
        label = self.labels.get(f'datalabel_{widget_number}')
        if canvas and label:
            canvas.axes.clear()
            canvas.axes.plot(x, y)
            canvas.draw()
            label.setText(title)

    def set_line_plot(self, data: list, widget_number: int, title: str):
        canvas: MplCanvas = self.canvases.get(f'widget_{widget_number}')
        label = self.labels.get(f'datalabel_{widget_number}')
        if canvas and label:
            canvas.axes.clear()
            canvas.axes.plot(data)
            canvas.draw()
            label.setText(title)

    def set_scatter_plot(self, data: list, widget_number: int, title: str):
        canvas: MplCanvas = self.canvases.get(f'widget_{widget_number}')
        label = self.labels.get(f'datalabel_{widget_number}')
        if canvas and label:
            canvas.axes.clear()
            canvas.axes.scatter(range(len(data)), data)
            canvas.draw()
            label.setText(title)

    def clear_plots(self):
        for i in range(1, 13):
            canvas = self.canvases.get(f'widget_{i}')
            label = self.labels.get(f'datalabel_{i}')
            if canvas:
                canvas.axes.clear()
                canvas.draw()
            if label:
                label.setText("")
