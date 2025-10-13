#!/usr/bin/env python

import os
import sys
from pathlib import Path

import rospy
from PySide2.QtQml import QQmlApplicationEngine
from PySide2.QtWidgets import QApplication
from PySide2.QtGui import QIcon
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, VFR_HUD
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu

from source.backEnd import backEnd
from source.joystickHandler import JoystickHandler
from wing_modules.CameraInterface.CameraCaptureInterfaceImplementation.FfmpegCameraFrameCapture import \
    FfmpegCameraFrameCapture
from wing_modules.CameraInterface.CameraCaptureInterfaceImplementation.GiCameraFrameCapture import GiCameraFrameCapture
from wing_modules.CameraInterface.CameraCaptureInterfaceImplementation.OpencvCameraFrameCapture import \
    OpencvCameraFrameCapture
from wing_modules.CameraInterface.CameraCaptureInterfaceImplementation.OpencvGstBackedCameraFrameCapture import \
    OpencvGstBackedCameraFrameCapture
from wing_modules.CameraInterface.FrameProcessor import FrameProcessor

if __name__ == "__main__":

    rospy.init_node("FieldTestAppNode", anonymous=True)
    # Avoids the warning of material style is not found.
    os.environ["QT_QUICK_CONTROLS_STYLE"] = "Material"

    app = QApplication(sys.argv)
    qml_path = Path(__file__).resolve().parent / "qml"
    app_icon = QIcon()
    app_icon.addFile(str(qml_path / "icon.png"))
    app.setWindowIcon(app_icon)
    engine = QQmlApplicationEngine()

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
        {"topicName": "/funnywing/rescueStatus", "dataType": Bool, "callbackType": "rescueStatus"},
        {"topicName": "/funnywing/orientation", "dataType": Imu, "callbackType": "funnywingOrientation"},
        {"topicName": "/funnywing/vfrHud", "dataType": VFR_HUD, "callbackType": "funnywingvfrHud"}
    ]

    backend = backEnd(engine, dataSubscriptionConfig, sysId, compId, tgSysId, tgCompId)

    # Setting up the camera monitor display. I think, It Must be done after engine loaded the QML app,
    # otherwise the signals emitted by provider to the front-end, cause segfault error, since their
    # corresponding QML sides are not loaded and actually this leads to accessing to (I think!) uninitialized
    # memory parts and so segfault error.
    frame_processor = FrameProcessor(track_topic="/funnywing/track")
    # This is necessary to get the last track id which is determined in the FrameProcessor, so we can lock on the last
    # track.
    backend.setupConnectionWithFrameProcessor(frame_processor)
    # TODO: clean up here after through field tests
    # For gazebo simulation camera use: rtsp://127.0.0.1:8554/test
    # Uncomment or select capture based on your camera
    # For runcam6
    # cameraFrameCapture = GiCameraFrameCapture(frame_source="rtsp://192.168.1.150:554/stream0")
    # For univision camera
    # cameraFrameCapture = GiCameraFrameCapture(frame_source="rtsp://admin:admin123456789#@192.168.1.68:554/\#\!/ipc/live")
    # For runcam6
    # cameraFrameCapture = FfmpegCameraFrameCapture(frame_source="rtsp://192.168.1.150:554/stream0")
    # For univision camera
    # cameraFrameCapture = FfmpegCameraFrameCapture(frame_source="rtsp://admin:admin123456789#@192.168.1.68:554/\#\!/ipc/live")
    # For runcam6 camera
    # cameraFrameCapture = OpencvGstBackedCameraFrameCapture(frame_source="rtsp://192.168.1.150:554/stream0")
    # For univision camera
    # cameraFrameCapture = OpencvGstBackedCameraFrameCapture(
    #     frame_source="rtsp://admin:admin123456789#@192.168.1.68:554/\#\!/ipc/live")
    # For gazebo simulation camera
    cameraFrameCapture = OpencvCameraFrameCapture(frame_source="rtsp://127.0.0.1:8554/test")
    # For runcam6 camera
    # cameraFrameCapture = OpencvCameraFrameCapture(frame_source="rtsp://192.168.1.150:554/stream0")
    # For univision camera
    # cameraFrameCapture = OpencvCameraFrameCapture(frame_source="rtsp://admin:admin123456789#@192.168.1.68:554/\#\!/ipc/live")
    cameraFrameCapture.set_frame_processor(frame_processor)
    backend.createAndSetupFrameProvider(cameraFrameCapture, app)
    ################################################################################################
    sys.exit(app.exec_())
