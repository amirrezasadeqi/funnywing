#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import sys
import datetime
import os
from pathlib import Path
import rospy

from PySide2.QtGui import QGuiApplication
from PySide2.QtQml import QQmlApplicationEngine
from PySide2.QtCore import QObject, Signal, Slot, QTimer, QUrl
from PySide2.QtWidgets import QApplication


class MainWindow(QObject):

    def __init__(self):
        QObject.__init__(self)
        return


if __name__ == "__main__":

    rospy.init_node("FieldTestAppNode", anonymous=True)

    app = QApplication(sys.argv)
    engine = QQmlApplicationEngine()
    main = MainWindow()
    engine.rootContext().setContextProperty("backend", main)
    qml_file = Path(__file__).resolve().parent / "qml/FieldTestAppMain.qml"
    engine.load(str(qml_file))
    if not engine.rootObjects():
        sys.exit(-1)
    sys.exit(app.exec_())

