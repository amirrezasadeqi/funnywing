#!/usr/bin/env python

import sys
from pathlib import Path
import rospy

from PySide2.QtQml import QQmlApplicationEngine
from PySide2.QtCore import QObject, Signal, Slot
from PySide2.QtWidgets import QApplication


class backEnd(QObject):
    # Arguments are optional and are the name of function arguments in the QML side, e.g. in onDemand(val), val would be
    # the entry in arguments list below.
    setTargetGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setWingGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setWingVelocity = Signal(float, float, float, arguments=['vx', 'vy', 'vz'])
    setWingHeading = Signal(float, arguments=['hdg'])
    setWingFlightState = Signal(str, arguments=['flightState'])
    setWingRelAlt = Signal(float, arguments=['alt'])

    @Slot(bool)
    def setArmState(self, armState):
        self._armState = armState
        ###
        print(armState)
        return

    @Slot(str)
    def setFlightMode(self, flightMode):
        self._flightMode = flightMode
        ###
        print(flightMode)
        return

    @Slot(float, float, float)
    def goToLocation(self, lat, lon, alt):
        print(lat, lon, alt)
        return

    def __init__(self):
        super().__init__()
        self._armState = False
        self._flightMode = "MANUAL"
        return


if __name__ == "__main__":

    rospy.init_node("FieldTestAppNode", anonymous=True)

    app = QApplication(sys.argv)
    engine = QQmlApplicationEngine()
    # Closing also the back-end when user closes the front-end.
    engine.quit.connect(app.quit)

    qml_file = Path(__file__).resolve().parent / "qml/FieldTestAppMain.qml"
    engine.load(str(qml_file))
    if not engine.rootObjects():
        sys.exit(-1)

    backend = backEnd()
    engine.rootContext().setContextProperty("backend", backend)

    sys.exit(app.exec_())
