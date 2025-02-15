#!/usr/bin/env python3

from argparse import ArgumentParser

import rospy
from rospkg import RosPack as rospack
from wing_navigator.srv import SetDouble, SetDoubleRequest, SetDoubleResponse, GetDouble, GetDoubleRequest, \
    GetDoubleResponse

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
        self._get_camera_zoom_service = rospy.Service("/funnywing/camera/get_camera_zoom", GetDouble,
                                                      self._get_camera_zoom_service_handler)
        return

    def _set_preset_index_service_handler(self, request: SetDoubleRequest):
        return SetDoubleResponse(self._camera_controller.set_preset_at_idx(request.data))

    def _get_camera_zoom_service_handler(self, request: GetDoubleRequest):
        camera_zoom = self._camera_controller.get_zoom()
        if camera_zoom is None:
            # camera zoom is not available.
            return GetDoubleResponse(-1)
        else:
            return GetDoubleResponse(camera_zoom)


def main():
    rospy.init_node("camera_control_node", anonymous=True)
    arg_parser = ArgumentParser()
    arg_parser.add_argument("-t", "--camera_type", type=str, default="gazebo_ros",
                            help="Type of the camera. possible options are 'gazebo_ros', 'tamron' and 'univision'")
    arg_parser.add_argument("-p", "--preset_table_file", type=str,
                            default=rospack().get_path("wing_navigator") + "/Configs/tamron_preset_table.csv",
                            help="Path of the CSV file containing the preset table.")
    args = arg_parser.parse_args()
    if "gazebo_ros" == args.camera_type:
        camera_controller = GazeboROSCameraController((1, 10), "/front_camera/zoom_camera_plugin/set_camera_zoom",
                                                      "/front_camera/zoom_camera_plugin/get_camera_zoom")
    elif "tamron" == args.camera_type:
        camera_controller = TamronCameraController((1, 10), port="/dev/ttyUSB0", baudrate=9600,
                                                   preset_table_file=args.preset_table_file)
    else:
        rospy.logerr(f"{args.camera_type} camera controller is not implemented yet!")
        return
    camera_controller_proxy = CameraControllerProxy(camera_controller=camera_controller)
    rospy.loginfo("Listening for Camera Control Requests ...")
    rospy.spin()


if __name__ == "__main__":
    main()
