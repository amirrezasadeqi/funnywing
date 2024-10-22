#!/usr/bin/env python

import os
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
from wing_modules.CameraMonitorFrameProvider import CameraMonitorFrameProvider
from wing_modules.OpencvCameraFrameCapture import OpencvCameraFrameCapture

if __name__ == "__main__":

    rospy.init_node("FieldTestAppNode", anonymous=True)
    # Avoids the warning of material style is not found.
    os.environ["QT_QUICK_CONTROLS_STYLE"] = "Material"

    app = QApplication(sys.argv)
    engine = QQmlApplicationEngine()

    qml_file = Path(__file__).resolve().parent / "qml/FieldTestAppMain.qml"
    engine.load(str(qml_file))
    if not engine.rootObjects():
        sys.exit(-1)

    backFrontConnections = backFrontEndCommunication()
    engine.rootContext().setContextProperty("backFrontConnections", backFrontConnections)
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

    # Setting up the camera monitor display. I think, It Must be done after engine loaded the QML app,
    # otherwise the signals emitted by provider to the front-end, cause segfault error, since their
    # corresponding QML sides are not loaded and actually this leads to accessing to (I think!) uninitialized
    # memory parts and so segfault error.
    cameraFrameCapture = OpencvCameraFrameCapture(frame_source="rtsp://127.0.0.1:8554/stream")
    cameraMonitorFrameProvider = CameraMonitorFrameProvider(frame_capture=cameraFrameCapture,
                                                            backFrontConnection=backFrontConnections)
    engine.addImageProvider("cameraMonitorFrameProvider", cameraMonitorFrameProvider)
    # close.accepted = false on QML front-end, holds the front-end up till the back-end to be closed. so,
    # we need to close the app using its quit slot, after the camera monitor frame provider was stopped.
    cameraMonitorFrameProvider.cameraMonitorFrameProviderQuited.connect(app.quit)

    sys.exit(app.exec_())
