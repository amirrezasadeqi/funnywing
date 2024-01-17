import threading

import numpy as np
import pymap3d
import rospy
from PySide2.QtCore import QObject
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool

from wing_modules.EllipsoidMSLConversion import EllipsoidMSLConversion


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
        self._ellipsoidMSLConverter = EllipsoidMSLConversion()
        self._lastWingGlobalPose = None
        self._lastTargetGlobalPose = None

        # ROS Timer to update distance to target
        self._distToTgUpdaterTimer = rospy.Timer(rospy.Duration(0, int((1.0 / 5.0) * 1e9)), self._updateDistToTg)
        # create listener thread to spin
        self._rosSpinnerThread = threading.Thread(target=self._rosSpinnerThreadCallback)
        # start the thread
        self._rosSpinnerThread.start()
        return

    def _setupCallbackTypeMap(self):
        self._callbackTypeMap = {
            "funnywingState": self._stateCallback,
            "funnywingGlobalPosition": self._globalPositionCallback,
            "funnywingGpsVelocity": self._gpsVelocityCallback,
            "funnywingGpsHeading": self._gpsHeadingCallback,
            "funnywingGpsRelativeAltitude": self._gpsRelAltCallback,
            "targetGlobalPosition": self._tgGlobalPositionCallback,
            "virtualTargetGlobalPosition": self._virtTgGlobalPosCallback,
            "rescueStatus": self._rescueStatusCallback
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
        rospy.spin()
        return

    def _stateCallback(self, msg: State):
        self._backFrontConnection.setWingFlightState.emit(msg.mode)
        return

    def _globalPositionCallback(self, msg: NavSatFix):
        self._lastWingGlobalPose = [msg.latitude, msg.longitude, msg.altitude]
        self._backFrontConnection.setWingGPS.emit(self._lastWingGlobalPose[0],
                                                  self._lastWingGlobalPose[1],
                                                  self._lastWingGlobalPose[2])
        return

    def _gpsVelocityCallback(self, msg: TwistStamped):
        self._backFrontConnection.setWingVelocity.emit(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        return

    def _gpsHeadingCallback(self, msg: Float64):
        self._backFrontConnection.setWingHeading.emit(msg.data)
        return

    def _gpsRelAltCallback(self, msg: Float64):
        self._backFrontConnection.setWingRelAlt.emit(msg.data)
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
