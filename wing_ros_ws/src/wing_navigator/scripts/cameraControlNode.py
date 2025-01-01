#!/usr/bin/env python3

from argparse import ArgumentParser

import rospy
from wing_navigator.srv import SetDouble, SetDoubleRequest, SetDoubleResponse

from wing_modules.CameraInterface.CameraControlInterface import CameraControlInterface
from wing_modules.CameraInterface.CameraControlInterfaceImplementation.GazeboROSCameraController import \
    GazeboROSCameraController
from wing_modules.CameraInterface.CameraControlInterfaceImplementation.TamronCameraController import \
    TamronCameraController


class CameraControllerProxy:
    def __init__(self, camera_controller: CameraControlInterface):
        self._camera_controller = camera_controller
        self._set_preset_index_service = rospy.Service("/funnywing/camera/set_preset_index", SetDouble,
                                                       self._set_preset_index_service_handler)
        return

    def _set_preset_index_service_handler(self, request: SetDoubleRequest):
        return SetDoubleResponse(self._camera_controller.set_preset_at_idx(request.data))


def main():
    rospy.init_node("camera_control_node", anonymous=True)
    arg_parser = ArgumentParser()
    arg_parser.add_argument("-t", "--camera_type", type=str, default="gazebo_ros",
                            help="Type of the camera. possible options are 'gazebo_ros', 'tamron' and 'univision'")
    args = arg_parser.parse_args()
    if "gazebo_ros" == args.camera_type:
        camera_controller = GazeboROSCameraController((1, 10), "/front_camera/zoom_camera_plugin/set_camera_zoom")
    elif "tamron" == args.camera_type:
        camera_controller = TamronCameraController((1, 10), port="/dev/ttyUSB0", baudrate=9600)
    else:
        rospy.logerr(f"{args.camera_type} camera controller is not implemented yet!")
        return
    camera_controller_proxy = CameraControllerProxy(camera_controller=camera_controller)
    rospy.loginfo("Listening for Camera Control Requests ...")
    rospy.spin()


if __name__ == "__main__":
    main()
