import math

from PySide2.QtCore import QObject, Signal, Slot
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu


class backFrontEndCommunication(QObject):
    # List of back to front end signals
    # Arguments are optional and are the name of function arguments in the QML side, e.g. in onDemand(val), val would be
    # the entry in arguments list below.
    setTargetGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setVirtualTargetGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setWingGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setWingVelocity = Signal(float, float, float, arguments=['vx', 'vy', 'vz'])
    setWingHeading = Signal(float, arguments=['hdg'])
    setWingAttitude = Signal(float, float, float, arguments=['roll', 'pitch', 'yaw'])
    setWingFlightState = Signal(str, arguments=['flightState'])
    setWingRelAlt = Signal(float, arguments=['alt'])
    setDistanceToTarget = Signal(float, arguments=['dist'])
    showRescueStatus = Signal(bool, arguments=['rescueStatus'])
    setWingRecvDataRate = Signal(float, arguments=['rate'])
    setTgRecvDataRate = Signal(float, arguments=['rate'])
    updateCameraMonitorFrame = Signal()
    setWingThrottle = Signal(float, arguments=['throttle'])
    setWingVoltage = Signal(float, arguments=['voltage'])
    setFlightTime = Signal(str, arguments=['flightTime'])
    setGroundSpeed = Signal(float, arguments=['groundspeed'])
    setAirSpeed = Signal(float, arguments=['airspeed'])

    # List of back end internal signals
    setArmStateSignal = Signal(bool)
    setFlightModeSignal = Signal(str)
    goToLocationSignal = Signal(float, float, float)
    sendSetRescueStatusSignal = Signal(bool)
    handleTestScenarioSignal = Signal(int, bool)
    setSimpleTrackerSettingsSignal = Signal(float, bool, bool)
    setSimpleTrackerActivationSignal = Signal(bool)
    setArduplaneParamSignal = Signal(str, float)
    closeBackendSignal = Signal()
    setZoomPercentageSignal = Signal(int)
    trackLockSignal = Signal(bool, int)
    setVisualTrackerSettingsSignal = Signal(float, int, int)
    setCameraBasedGuiderConfigsSignal = Signal('QVariantMap')
    setLastTrackIdSignal = Signal(int)  # back-end internal signal
    
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

    @Slot(bool)
    def sendSetRescueStatus(self, rescueState):
        self.sendSetRescueStatusSignal.emit(rescueState)
        return

    @Slot(int, bool)
    def handleTestScenario(self, scenarioIdx, active):
        self.handleTestScenarioSignal.emit(scenarioIdx, active)
        return

    @Slot(float, bool, bool)
    def setSimpleTrackerSettings(self, waypointRadius, local, wingAsVirtualCenter):
        self.setSimpleTrackerSettingsSignal.emit(waypointRadius, local, wingAsVirtualCenter)
        return

    @Slot(bool)
    def setSimpleTrackerActivation(self, active):
        self.setSimpleTrackerActivationSignal.emit(active)
        return

    @Slot(str, float)
    def setArduplaneParam(self, paramName, paramValue):
        self.setArduplaneParamSignal.emit(paramName, paramValue)
        return

    @Slot()
    def closeBackend(self):
        self.closeBackendSignal.emit()
        return

    @Slot(int)
    def setZoomPercentage(self, zoom_percentage):
        self.setZoomPercentageSignal.emit(zoom_percentage)
        return

    @Slot(bool, int)
    def setTrackLockState(self, locked, track_id):
        self.trackLockSignal.emit(locked, track_id)
        return

    @Slot(float, int, int)
    def setVisualTrackerSettings(self, distThresh, initDelay, hitCountMax):
        self.setVisualTrackerSettingsSignal.emit(distThresh, initDelay, hitCountMax)
        return

    @Slot('QVariantMap')
    def setCameraBasedGuiderConfigs(self, configs):
        self.setCameraBasedGuiderConfigsSignal.emit(configs)
        return
