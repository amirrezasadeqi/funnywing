#!/usr/bin/env python

import sys
import math
from pathlib import Path
import rospy
import threading
from pymavlink import mavutil
from mavros import mavlink

from mavros_msgs.msg import Mavlink, State
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

from PySide2.QtQml import QQmlApplicationEngine
from PySide2.QtCore import QObject, Signal, Slot
from PySide2.QtWidgets import QApplication


class backFrontEndCommunication(QObject):
    # List of back to front end signals
    # Arguments are optional and are the name of function arguments in the QML side, e.g. in onDemand(val), val would be
    # the entry in arguments list below.
    setTargetGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setWingGPS = Signal(float, float, float, arguments=['lat', 'lon', 'alt'])
    setWingVelocity = Signal(float, float, float, arguments=['vx', 'vy', 'vz'])
    setWingHeading = Signal(float, arguments=['hdg'])
    setWingFlightState = Signal(str, arguments=['flightState'])
    setWingRelAlt = Signal(float, arguments=['alt'])

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


class backEnd(QObject):
    def __init__(self, dataSubscriptionConfig, backFrontConnection, systemID, componentID, tgSystemID, tgComponentID):
        super().__init__()
        self._dataSubscriptionConfig = dataSubscriptionConfig
        self._systemID = systemID
        self._componentID = componentID
        self._tgSystemID = tgSystemID
        self._tgComponentID = tgComponentID

        # Setup signals and connections
        self._backFrontConnection = backFrontConnection
        self._backFrontConnection.setArmStateSignal.connect(self.pubArmDisarmCommand)
        self._backFrontConnection.setFlightModeSignal.connect(self.pubSetModeCommand)
        self._backFrontConnection.goToLocationSignal.connect(self.pubGoToCommand)

        self._dataUpdater = dataUpdater(self._dataSubscriptionConfig, self._backFrontConnection)
        # TODO[test needed]: MAVLink object does not try to connect to the connection string and
        #   I don't know if the connection string is important in de/serialization. So I will use
        #   empty string now and if it will be ok delete this, otherwise we must pass the address
        #   of the connection(since MAVLink does not connect to that automatically, I think there
        #   will be no problem about occupied connection).
        self._protocolObj = mavutil.mavlink.MAVLink('', self._systemID, self._componentID)
        # Publisher for sending mavlink Commands and all the data which is needed in the RPI side.
        self._toRfComPublisher = rospy.Publisher("/GCS/from", Mavlink, queue_size=10)
        return

    ARDUPLANE_MODE_MAP = {
        "MANUAL": 0,
        "CIRCLE": 1,
        "STABILIZE": 2,
        "TRAINING": 3,
        "ACRO": 4,
        "FBWA": 5,
        "FBWB": 6,
        "CRUISE": 7,
        "AUTOTUNE": 8,
        "AUTO": 10,
        "RTL": 11,
        "LOITER": 12,
        "GUIDED": 15
    }

    @Slot(bool)
    def pubArmDisarmCommand(self, armState):
        # create mavlink message
        mavMsg = mavutil.mavlink.MAVLink_command_long_message(self._tgSystemID, self._tgComponentID,
                                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                                              1 if armState else 0, 0, 0, 0, 0, 0, 0)
        # convert it to mavros_msgs/Mavlink message
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        # publish the message, and it will automatically be sent.
        self._toRfComPublisher.publish(rosMsg)
        ###
        print(armState)
        return

    @Slot(str)
    def pubSetModeCommand(self, flightMode):
        mavMsg = mavutil.mavlink.MAVLink_command_long_message(self._tgSystemID, self._tgComponentID,
                                                              mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                                              0, self.ARDUPLANE_MODE_MAP[flightMode], 0, 0, 0, 0, 0)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        ###
        print(flightMode)
        return

    @Slot(float, float, float)
    def pubGoToCommand(self, lat, lon, alt):
        # Scaling lat, lon to use them with MAVLink_command_int_message.
        lat = int(lat * 1e7)
        lon = int(lon * 1e7)
        mavMsg = mavutil.mavlink.MAVLink_command_int_message(self._tgSystemID, self._tgComponentID,
                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                             mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1,
                                                             mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 120,
                                                             0, lat, lon, alt)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        print(lat, lon, alt)
        return


if __name__ == "__main__":

    rospy.init_node("FieldTestAppNode", anonymous=True)

    app = QApplication(sys.argv)
    backFrontConnections = backFrontEndCommunication()
    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty("backFrontConnections", backFrontConnections)
    # Closing also the back-end when user closes the front-end.
    engine.quit.connect(app.quit)

    qml_file = Path(__file__).resolve().parent / "qml/FieldTestAppMain.qml"
    engine.load(str(qml_file))
    if not engine.rootObjects():
        sys.exit(-1)

    ################################################################################################
    # Back-End Tasks Codes
    ################################################################################################
    sysId = mavutil.mavlink.MAV_TYPE_GCS  # MAVLink ID for GCS
    compId = 1
    tgSysId = mavutil.mavlink.MAV_TYPE_FIXED_WING
    tgCompId = 1

    dataSubscriptionConfig = [
        {"topicName": "/funnywing/state", "dataType": State, "callbackType": "funnywingState"},
        {"topicName": "/funnywing/globalPosition", "dataType": NavSatFix, "callbackType": "funnywingGlobalPosition"},
        {"topicName": "/funnywing/gpsVelocity", "dataType": TwistStamped, "callbackType": "funnywingGpsVelocity"},
        {"topicName": "/funnywing/gpsHeading", "dataType": Float64, "callbackType": "funnywingGpsHeading"},
        {"topicName": "/funnywing/gpsRelativeAltitude", "dataType": Float64,
         "callbackType": "funnywingGpsRelativeAltitude"},
        {"topicName": "/target/globalPosition", "dataType": NavSatFix, "callbackType": "targetGlobalPosition"}
    ]

    ################################################################################################
    # TODO: delete this. use it just for test.
    # dataSubscriptionConfig = [
    #     {"topicName": "/mavros/state", "dataType": State, "callbackType": "funnywingState"},
    #     {"topicName": "/mavros/global_position/global", "dataType": NavSatFix,
    #      "callbackType": "funnywingGlobalPosition"},
    #     {"topicName": "/mavros/global_position/raw/gps_vel", "dataType": TwistStamped,
    #      "callbackType": "funnywingGpsVelocity"},
    #     {"topicName": "/mavros/global_position/compass_hdg", "dataType": Float64,
    #      "callbackType": "funnywingGpsHeading"},
    #     {"topicName": "/mavros/global_position/rel_alt", "dataType": Float64,
    #      "callbackType": "funnywingGpsRelativeAltitude"},
    #     {"topicName": "/target/globalPosition", "dataType": NavSatFix, "callbackType": "targetGlobalPosition"}
    # ]
    ################################################################################################

    backend = backEnd(dataSubscriptionConfig, backFrontConnections, sysId, compId, tgSysId, tgCompId)
    ################################################################################################

    sys.exit(app.exec_())
