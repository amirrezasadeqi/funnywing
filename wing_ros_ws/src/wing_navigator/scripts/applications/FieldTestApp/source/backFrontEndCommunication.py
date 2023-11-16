import math

from PySide2.QtCore import QObject, Signal, Slot


class backFrontEndCommunication(QObject):
    # List of back to front end signals
    # Arguments are optional and are the name of function arguments in the QML side, e.g. in onDemand(val), val would be
    # the entry in arguments list below.
    setTargetGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setVirtualTargetGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setWingGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setWingVelocity = Signal(float, float, float, arguments=['vx', 'vy', 'vz'])
    setWingHeading = Signal(float, arguments=['hdg'])
    setWingFlightState = Signal(str, arguments=['flightState'])
    setWingRelAlt = Signal(float, arguments=['alt'])
    setDistanceToTarget = Signal(float, arguments=['dist'])

    # List of back end internal signals
    setArmStateSignal = Signal(bool)
    setFlightModeSignal = Signal(str)
    goToLocationSignal = Signal(float, float, float)

    def __init__(self):
        super().__init__()
        return

    @Slot(bool)
    def setArmState(self, armState):
        self.setArmStateSignal.emit(armState)
        return

    @Slot(str)
    def setFlightMode(self, flightMode):
        self.setFlightModeSignal.emit(flightMode)
        return

    @Slot(float, float, float)
    def goToLocation(self, lat, lon, alt):
        for val in [lat, lon, alt]:
            if math.isnan(val):
                print("Please Enter Valid GPS Location!")
                return
        self.goToLocationSignal.emit(lat, lon, alt)
        return
