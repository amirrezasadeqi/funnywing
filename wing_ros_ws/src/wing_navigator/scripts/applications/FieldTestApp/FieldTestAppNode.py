#!/usr/bin/env python

import sys
from pathlib import Path

import rospy
from PySide2.QtQml import QQmlApplicationEngine
from PySide2.QtWidgets import QApplication
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool

from source.backEnd import backEnd
from source.backFrontEndCommunication import backFrontEndCommunication

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
        {"topicName": "/target/globalPosition", "dataType": NavSatFix, "callbackType": "targetGlobalPosition"},
        {"topicName": "/virtualTarget/globalPosition", "dataType": NavSatFix,
         "callbackType": "virtualTargetGlobalPosition"},
        {"topicName": "/funnywing/rescueStatus", "dataType": Bool, "callbackType": "rescueStatus"}
    ]

    backend = backEnd(dataSubscriptionConfig, backFrontConnections, sysId, compId, tgSysId, tgCompId)
    ################################################################################################

    sys.exit(app.exec_())
