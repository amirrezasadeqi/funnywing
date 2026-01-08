import threading
import math

import numpy as np
import pymap3d
import rospy
import rostopic
from PySide2.QtCore import QObject
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, VFR_HUD
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu, BatteryState
from wing_modules.EllipsoidMSLConversion import EllipsoidMSLConversion
from scipy.spatial.transform import Rotation as R

class dataUpdater(QObject):
    def __init__(self, dataSubConfig, backFrontConnection):
        super().__init__()

        self._dataSubConfig = dataSubConfig
        self._backFrontConnection = backFrontConnection

        # Create Subscribers
        self._subscriberList = []
        self._callbackTypeMap = {}
        self._setupCallbackTypeMap()
        self._createSubscriptions()
        # Set up the monitor mechanism for the rates of target GPS and wing mavlink data streams.
        self._createDataRateMonitors()
        self._ellipsoidMSLConverter = EllipsoidMSLConversion()
        self._lastWingGlobalPose = None
        self._lastTargetGlobalPose = None
        
        self._isArmed = False
        self._flightStartTimeSec = 0.0
        self._currentFlightTimeSec = 0.0

        # ROS Timer to update data rate monitors
        self._dataRateUpdaterTimer = rospy.Timer(rospy.Duration(secs=0, nsecs=500000000), self._updateDataRateMonitors)
        # ROS Timer to update distance to target
        self._distToTgUpdaterTimer = rospy.Timer(rospy.Duration(0, int((1.0 / 5.0) * 1e9)), self._updateDistToTg)
        # create listener thread to spin
        self._rosSpinnerThread = threading.Thread(target=self._rosSpinnerThreadCallback)
        self._stopSpinnerThread = False
        # start the thread
        self._rosSpinnerThread.start()
        return

    def stop(self):
        self._stopSpinnerThread = True
        self._rosSpinnerThread.join()
        self._distToTgUpdaterTimer.shutdown()
        self._dataRateUpdaterTimer.shutdown()
        return

    def _setupCallbackTypeMap(self):
        self._callbackTypeMap = {
            "funnywingState": self._stateCallback,
            "funnywingGlobalPosition": self._globalPositionCallback,
            "funnywingGpsVelocity": self._gpsVelocityCallback,
            "funnywingGpsHeading": self._gpsHeadingCallback,
            "funnywingGpsRelativeAltitude": self._gpsRelAltCallback,
            "funnywingOrientation": self._orientationCallback,
            "funnywingBatteryState": self._batteryStateCallback,
            "funnywingvfrHud": self._vfrHudCallback,
            "targetGlobalPosition": self._tgGlobalPositionCallback,
            "virtualTargetGlobalPosition": self._virtTgGlobalPosCallback,
            "rescueStatus": self._rescueStatusCallback,
        }
        return
 
    def _createSubscriptions(self):
        # create ROS listeners to get latest data and send it to frontend
        for config in self._dataSubConfig:
            self._subscriberList.append({"topicName": config["topicName"],
                                         "subscriber": rospy.Subscriber(config["topicName"], config["dataType"],
                                                                        callback=self._callbackTypeMap[
                                                                            config["callbackType"]])})
        return

    def _rosSpinnerThreadCallback(self):
        while not rospy.is_shutdown() and not self._stopSpinnerThread:
            rospy.sleep(0.5)
        return

    def _globalPositionCallback(self, msg: NavSatFix):
        self._lastWingGlobalPose = [msg.latitude, msg.longitude, msg.altitude]
        self._backFrontConnection.setWingGPS.emit(self._lastWingGlobalPose[0],
                                                  self._lastWingGlobalPose[1],
                                                  self._lastWingGlobalPose[2])
        return
    
    def _orientationCallback(self, msg: Imu):
        q = msg.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])

        forward_vector = [1, 0, 0]
        right_vector = [0, 1, 0]
        down_vector = [0, 0, 1]

        forward_rotated = r.apply(forward_vector)
        right_rotated = r.apply(right_vector)
        down_rotated = r.apply(down_vector)

        yaw_rad = math.atan2(right_rotated[2], down_rotated[2])
        pitch_rad = -math.asin(forward_rotated[2])

        roll_rad = math.atan2(forward_rotated[1], forward_rotated[0])
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        yaw_deg = math.degrees(yaw_rad)
    
        yaw_deg = (yaw_deg + 360) % 360

        self._backFrontConnection.setWingAttitude.emit(roll_deg, pitch_deg, yaw_deg)

        
    def _gpsVelocityCallback(self, msg: TwistStamped):
        self._backFrontConnection.setWingVelocity.emit(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        return

    def _gpsHeadingCallback(self, msg: Float64):
        self._backFrontConnection.setWingHeading.emit(msg.data)
        return

    def _gpsRelAltCallback(self, msg: Float64):
        self._backFrontConnection.setWingRelAlt.emit(msg.data)
        return
    
    def _vfrHudCallback(self, msg: VFR_HUD):
        self._backFrontConnection.setAirSpeed.emit(msg.airspeed)
        self._backFrontConnection.setWingThrottle.emit(float(msg.throttle))
        self._backFrontConnection.setGroundSpeed.emit(msg.groundspeed)
        return
    
    def _batteryStateCallback(self, msg: BatteryState):
        self._backFrontConnection.setWingVoltage.emit(msg.voltage)
        return

    def _tgGlobalPositionCallback(self, msg: NavSatFix):
        self._lastTargetGlobalPose = [msg.latitude, msg.longitude, msg.altitude]
        self._backFrontConnection.setTargetGPS.emit(self._lastTargetGlobalPose[0],
                                                    self._lastTargetGlobalPose[1],
                                                    self._lastTargetGlobalPose[2])
        return

    def _virtTgGlobalPosCallback(self, msg: NavSatFix):
        self._backFrontConnection.setVirtualTargetGPS.emit(msg.latitude, msg.longitude, msg.altitude)
        return

    def _rescueStatusCallback(self, msg: Bool):
        self._backFrontConnection.showRescueStatus.emit(msg.data)
        return

    def _updateDistToTg(self, event=None):
        if None not in [self._lastWingGlobalPose, self._lastTargetGlobalPose]:
            self._backFrontConnection.setDistanceToTarget.emit(self._calculateDistance())
        else:
            rospy.loginfo(f"Please wait for last position of funnywing and target to be available!")
        return

    def _calculateDistance(self):
        tgGlobalWGS = self._ellipsoidMSLConverter.mslToEllipsoid(self._lastTargetGlobalPose)
        fwGlobalWGS = self._ellipsoidMSLConverter.mslToEllipsoid(self._lastWingGlobalPose)

        tgLocalPos = pymap3d.geodetic2ecef(tgGlobalWGS[0], tgGlobalWGS[1], tgGlobalWGS[2])
        fwLocalPos = pymap3d.geodetic2ecef(fwGlobalWGS[0], fwGlobalWGS[1], fwGlobalWGS[2])
        diffVector = np.array(tgLocalPos) - np.array(fwLocalPos)

        return np.linalg.norm(diffVector)

    def _createDataRateMonitors(self):
        self._wingTopicHz = rostopic.ROSTopicHz(100)
        self._tgGPSTopicHz = rostopic.ROSTopicHz(100)
        self._subscriberList.append(rospy.Subscriber("/funnywing/from", rospy.AnyMsg, self._wingTopicHz.callback_hz,
                                                     callback_args="/funnywing/from"))
        self._subscriberList.append(
            rospy.Subscriber("/target/globalPosition", rospy.AnyMsg, self._tgGPSTopicHz.callback_hz,
                             callback_args="/target/globalPosition"))
        return
    
    def _stateCallback(self, msg: State):
        try:
            self._backFrontConnection.setWingFlightState.emit(msg.mode)
            self._updateFlightTime(msg)
        except Exception:
            rospy.logdebug("Couldn't emit setWingFlightState")
        return

    def _updateFlightTime(self, current_state: State):
        isCurrentlyArmed = current_state.armed
        
        if isCurrentlyArmed and not self._isArmed:
            self._isArmed = True
            self._flightStartTimeSec = rospy.get_time()
            self._currentFlightTimeSec = 0.0
        elif not isCurrentlyArmed and self._isArmed:
            self._isArmed = False
            self._flightStartTimeSec = 0.0
        if self._isArmed and self._flightStartTimeSec > 0:
            currentTime = rospy.get_time()
            self._currentFlightTimeSec = currentTime - self._flightStartTimeSec
            
            totalSeconds = int(self._currentFlightTimeSec)
            hours = totalSeconds // 3600
            minutes = (totalSeconds % 3600) // 60
            seconds = totalSeconds % 60
            formattedTime = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
            self._backFrontConnection.setFlightTime.emit(formattedTime)
        elif not self._isArmed:
            totalSeconds = int(self._currentFlightTimeSec)
            hours = totalSeconds // 3600
            minutes = (totalSeconds % 3600) // 60
            seconds = totalSeconds % 60
            formattedTime = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
            self._backFrontConnection.setFlightTime.emit(formattedTime)
        return
    
    def _updateDataRateMonitors(self, event=None):
        try:
            rate = self._wingTopicHz.get_hz("/funnywing/from")[0]
            self._backFrontConnection.setWingRecvDataRate.emit(rate)
        except Exception as e:
            rospy.loginfo("Wing data stream is down.")

        try:
            rate = self._tgGPSTopicHz.get_hz("/target/globalPosition")[0]
            self._backFrontConnection.setTgRecvDataRate.emit(rate)
        except Exception as e:
            rospy.loginfo("Target data stream is down.")

        return