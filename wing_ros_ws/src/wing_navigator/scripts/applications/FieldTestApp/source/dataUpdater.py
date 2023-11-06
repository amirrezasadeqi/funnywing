import threading

import rospy
from PySide2.QtCore import QObject
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


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
            "targetGlobalPosition": self._tgGlobalPositionCallback
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
        self._backFrontConnection.setWingGPS.emit(msg.latitude, msg.longitude, msg.altitude)
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
        self._backFrontConnection.setTargetGPS.emit(msg.latitude, msg.longitude, msg.altitude)
        return
