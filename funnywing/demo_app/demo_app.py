#!/usr/bin/env python

from PyQt5 import QtWidgets, uic
import sys
import subprocess as sp

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('demo_app.ui', self)

        self.button = self.findChild(QtWidgets.QPushButton, 'pushButton')
        self.button.clicked.connect(self.pushButtonPressed)
        
        self.lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit')
        self.lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_2')
        self.alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_3')

        self.show()

    def pushButtonPressed(self):
        sp.check_call(["./demo_app.bash", self.lat.text(), self.lon.text(), self.alt.text()])

app = QtWidgets.QApplication(sys.argv)
window = Ui()
app.exec_()

