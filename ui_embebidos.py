# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\Embedido_tarea2.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

from PyQt5 import QtCore, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(774, 836)
        self.label_30 = QtWidgets.QLabel(Dialog)
        self.label_30.setGeometry(QtCore.QRect(350, 130, 101, 21))
        self.label_30.setStyleSheet("color: rgb(0, 0, 0);\n\n")
        self.label_30.setObjectName("label_30")
        self.progressBar = QtWidgets.QProgressBar(Dialog)
        self.progressBar.setGeometry(QtCore.QRect(460, 130, 118, 23))
        self.progressBar.setProperty("value", 0)
        self.progressBar.setObjectName("progressBar")
        self.selec_12 = QtWidgets.QComboBox(Dialog)
        self.selec_12.setGeometry(QtCore.QRect(350, 160, 181, 31))
        self.selec_12.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.selec_12.setObjectName("selec_12")
        self.selec_12.addItem("")
        self.selec_12.addItem("")
        self.selec_12.addItem("")
        self.label_9 = QtWidgets.QLabel(Dialog)
        self.label_9.setGeometry(QtCore.QRect(120, 180, 81, 31))
        self.label_9.setObjectName("label_9")
        self.label_7 = QtWidgets.QLabel(Dialog)
        self.label_7.setGeometry(QtCore.QRect(120, 130, 81, 31))
        self.label_7.setObjectName("label_7")
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setGeometry(QtCore.QRect(350, 40, 71, 41))
        self.label_2.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_2.setFrameShape(QtWidgets.QFrame.Box)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.text_acc_sensibity = QtWidgets.QTextEdit(Dialog)
        self.text_acc_sensibity.setGeometry(QtCore.QRect(210, 180, 104, 31))
        self.text_acc_sensibity.setObjectName("text_acc_sensibity")
        self.selec_13 = QtWidgets.QComboBox(Dialog)
        self.selec_13.setGeometry(QtCore.QRect(360, 300, 181, 31))
        self.selec_13.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.selec_13.setObjectName("selec_13")
        self.selec_13.addItem("")
        self.selec_13.addItem("")
        self.label_31 = QtWidgets.QLabel(Dialog)
        self.label_31.setGeometry(QtCore.QRect(390, 270, 121, 21))
        self.label_31.setStyleSheet("color: rgb(0, 0, 0);\n\n")
        self.label_31.setObjectName("label_31")
        self.label_32 = QtWidgets.QLabel(Dialog)
        self.label_32.setGeometry(QtCore.QRect(170, 100, 121, 21))
        self.label_32.setStyleSheet("color: rgb(0, 0, 0);\n\n")
        self.label_32.setObjectName("label_32")
        self.label_33 = QtWidgets.QLabel(Dialog)
        self.label_33.setGeometry(QtCore.QRect(170, 220, 121, 21))
        self.label_33.setStyleSheet("color: rgb(0, 0, 0);\n\n")
        self.label_33.setObjectName("label_33")
        self.label_8 = QtWidgets.QLabel(Dialog)
        self.label_8.setGeometry(QtCore.QRect(120, 250, 81, 31))
        self.label_8.setObjectName("label_8")
        self.label_10 = QtWidgets.QLabel(Dialog)
        self.label_10.setGeometry(QtCore.QRect(120, 300, 81, 31))
        self.label_10.setObjectName("label_10")
        self.text_acc_sensibity_2 = QtWidgets.QTextEdit(Dialog)
        self.text_acc_sensibity_2.setGeometry(QtCore.QRect(210, 300, 104, 31))
        self.text_acc_sensibity_2.setObjectName("text_acc_sensibity_2")
        self.text_acc_sampling_2 = QtWidgets.QTextEdit(Dialog)
        self.text_acc_sampling_2.setGeometry(QtCore.QRect(210, 250, 104, 31))
        self.text_acc_sampling_2.setObjectName("text_acc_sampling_2")
        self.Plot1 = QtWidgets.QGraphicsView(Dialog)
        self.Plot1.setGeometry(QtCore.QRect(60, 420, 291, 181))
        self.Plot1.setFrameShape(QtWidgets.QFrame.Box)
        self.Plot1.setFrameShadow(QtWidgets.QFrame.Plain)
        self.Plot1.setObjectName("Plot1")
        self.Plot2 = QtWidgets.QGraphicsView(Dialog)
        self.Plot2.setGeometry(QtCore.QRect(390, 420, 291, 181))
        self.Plot2.setFrameShape(QtWidgets.QFrame.Box)
        self.Plot2.setFrameShadow(QtWidgets.QFrame.Plain)
        self.Plot2.setObjectName("Plot2")
        self.Plot3 = QtWidgets.QGraphicsView(Dialog)
        self.Plot3.setGeometry(QtCore.QRect(60, 640, 291, 181))
        self.Plot3.setFrameShape(QtWidgets.QFrame.Box)
        self.Plot3.setFrameShadow(QtWidgets.QFrame.Plain)
        self.Plot3.setObjectName("Plot3")
        self.Plot4 = QtWidgets.QGraphicsView(Dialog)
        self.Plot4.setGeometry(QtCore.QRect(390, 640, 291, 181))
        self.Plot4.setFrameShape(QtWidgets.QFrame.Box)
        self.Plot4.setFrameShadow(QtWidgets.QFrame.Plain)
        self.Plot4.setObjectName("Plot4")
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setGeometry(QtCore.QRect(120, 390, 151, 21))
        self.label_3.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_3.setFrameShape(QtWidgets.QFrame.Box)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(Dialog)
        self.label_4.setGeometry(QtCore.QRect(440, 390, 151, 21))
        self.label_4.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_4.setFrameShape(QtWidgets.QFrame.Box)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(Dialog)
        self.label_5.setGeometry(QtCore.QRect(120, 610, 151, 21))
        self.label_5.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_5.setFrameShape(QtWidgets.QFrame.Box)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(Dialog)
        self.label_6.setGeometry(QtCore.QRect(440, 610, 151, 21))
        self.label_6.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_6.setFrameShape(QtWidgets.QFrame.Box)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.pushButton = QtWidgets.QPushButton(Dialog)
        self.pushButton.setGeometry(QtCore.QRect(550, 160, 141, 31))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(Dialog)
        self.pushButton_2.setGeometry(QtCore.QRect(320, 370, 101, 41))
        self.pushButton_2.setObjectName("pushButton_2")
        self.comboBox_acc_sampling = QtWidgets.QComboBox(Dialog)
        self.comboBox_acc_sampling.setGeometry(QtCore.QRect(210, 130, 101, 31))
        self.comboBox_acc_sampling.setObjectName("comboBox_acc_sampling")
        self.comboBox_acc_sampling.addItem("")
        self.comboBox_acc_sampling.addItem("")
        self.comboBox_acc_sampling.addItem("")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "UI Sensores"))
        self.label_30.setText(_translate("Dialog", "<html><head/><body><p align=\"center\"><span style=\" text-decoration: underline;\">Sensor activo</span></p></body></html>"))
        self.selec_12.setItemText(0, _translate("Dialog", "<Ninguno>"))
        self.selec_12.setItemText(1, _translate("Dialog", "BMI270"))
        self.selec_12.setItemText(2, _translate("Dialog", "BMI688"))
        self.label_9.setText(_translate("Dialog", "Frecuencia de \nmuestro"))
        self.label_7.setText(_translate("Dialog", "Sensibilidad"))
        self.label_2.setText(_translate("Dialog", "Configuracion \nSensor"))
        self.selec_13.setItemText(0, _translate("Dialog", "Paralelo"))
        self.selec_13.setItemText(1, _translate("Dialog", "Forzado"))
        self.label_31.setText(_translate("Dialog", "Modo de Funcionamiento"))
        self.label_32.setText(_translate("Dialog", "<html><head/><body><p><span style=\" text-decoration: underline;\">Acelerómetro</span></p></body></html>"))
        self.label_33.setText(_translate("Dialog", "<html><head/><body><p><span style=\" text-decoration: underline;\">Giroscopio</span></p></body></html>"))
        self.label_8.setText(_translate("Dialog", "Sensibilidad"))
        self.label_10.setText(_translate("Dialog", "Frecuencia de \nmuestro"))
        self.label_3.setText(_translate("Dialog", "Datos 1: <Datos>"))
        self.label_4.setText(_translate("Dialog", "Datos 2: <Datos>"))
        self.label_5.setText(_translate("Dialog", "Datos 3: <Datos>"))
        self.label_6.setText(_translate("Dialog", "Datos 4: <Datos>"))
        self.pushButton.setText(_translate("Dialog", "Iniciar configuración"))
        self.pushButton_2.setText(_translate("Dialog", "Iniciar captación \nde datos"))
        self.comboBox_acc_sampling.setItemText(0, _translate("Dialog", "30"))
        self.comboBox_acc_sampling.setItemText(1, _translate("Dialog", "120"))
        self.comboBox_acc_sampling.setItemText(2, _translate("Dialog", "60"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())
