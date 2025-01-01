import rospy
from numpy import interp
from wing_navigator.srv import SetDouble, SetDoubleRequest

from wing_modules.CameraInterface.CameraControlInterface import CameraControlInterface


class GazeboROSCameraController(CameraControlInterface):
    def __init__(self, zoom_range: tuple, set_camera_zoom_service_name):
        """
        @param zoom_range: tuple of two elements, which represent the range of zoom values, like (1, 10).
        I think the tuple first value should not be less than 1, for example 0 is wrong, because the zoom
        level starts from 1, for example 1X(1 times), 2X and so on.
        @param set_camera_zoom_service_name: name of the service provided by the camera zoom plugin.
        format of the service name is: /<camera_name_in_sdf>/<plugin_name_in_sdf>/set_camera_zoom
        """
        super().__init__(zoom_range)
        rospy.wait_for_service(set_camera_zoom_service_name)
        self._set_camera_zoom_proxy = rospy.ServiceProxy(set_camera_zoom_service_name, SetDouble)
        return

    def set_zoom(self, zoom):
        if zoom > self._zoom_range[1]:
            zoom = self._zoom_range[1]
        elif zoom < self._zoom_range[0]:
            zoom = self._zoom_range[0]
        zoom_req = SetDoubleRequest(zoom)
        try:
            zoom_resp = self._set_camera_zoom_proxy(zoom_req)
            rospy.loginfo(f"Setting zoom level to {zoom} result: {zoom_resp.success}")
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return False

    def set_focus(self, focus):
        """
        This method is not implemented, since I think the simulated camera in gazebo uses the same mechanism for zooming
        and focusing. so, we don't use this method in the simulation.
        """
        rospy.logwarn("set_focus method is not implemented in GazeboROSCameraController")
        return

    def set_preset_at_idx(self, preset_idx, do_mapping=True):
        """
        Due to the fact that the simulated camera zoom is ideal(there is no blurring in zooming for example), we can use
        the set_zoom method for changing the preset.(Actually there is no need for changing the focus in the preset
        change in the simulation).
        Here preset_idx range is the same as the zoom range, for the current state of simulation. so don't worry about
        the preset table.
        """
        if do_mapping:
            # Mapping from 0-100 preset level to the raw range of preset values(here zoom range)
            preset_idx = interp(preset_idx, [0, 100], self._zoom_range)
        return self.set_zoom(preset_idx)
