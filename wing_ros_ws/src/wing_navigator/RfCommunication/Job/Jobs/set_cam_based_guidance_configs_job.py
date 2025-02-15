import rospy
from wing_navigator.srv import SetCameraBasedGuiderConfigs, SetCameraBasedGuiderConfigsRequest, \
    SetCameraBasedGuiderConfigsResponse

from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class set_cam_based_guidance_configs_job(JobInterface):
    _set_cam_based_guidance_configs_proxy = None

    def __init__(self, message, rfConnection: ConnectionInterface, system, component):
        """

        @type message: MAVLink_set_cam_based_guidance_configs_message
        """
        super().__init__(message, rfConnection, system, component)
        if set_cam_based_guidance_configs_job._set_cam_based_guidance_configs_proxy is None:
            rospy.wait_for_service("/funnywing/set_cam_based_guider_configs")
            set_cam_based_guidance_configs_job._set_cam_based_guidance_configs_proxy = rospy.ServiceProxy(
                "/funnywing/set_cam_based_guider_configs", SetCameraBasedGuiderConfigs)
        self._request = self._createRequest()
        self._response = SetCameraBasedGuiderConfigsResponse()
        return

    def _doJob(self):
        try:
            self._response = set_cam_based_guidance_configs_job._set_cam_based_guidance_configs_proxy(self._request)
            rospy.loginfo(f"Setting Camera Based Guidance Configs, Success: {self._response.success}")
        except Exception as e:
            rospy.logwarn(f"Exception occurred while setting Camera Based Guidance Configs: {e}")
        return

    def _createRequest(self) -> SetCameraBasedGuiderConfigsRequest:
        request = SetCameraBasedGuiderConfigsRequest()
        mavMsg = self.getMessage()
        request.profile_type = mavMsg.profile_type
        request.const_throttle = mavMsg.const_throttle
        request.x_pids = mavMsg.x_pids
        request.y_pids = mavMsg.y_pids
        request.a = mavMsg.a
        request.b = mavMsg.b
        request.wing_too_below_throttle = mavMsg.wing_too_below_throttle
        request.wing_too_above_throttle = mavMsg.wing_too_above_throttle
        request.wing_tg_at_same_level_throttle = mavMsg.wing_tg_at_same_level_throttle
        request.size_threshold = mavMsg.size_threshold
        request.wing_too_below_threshold = mavMsg.wing_too_below_threshold
        request.wing_too_above_threshold = mavMsg.wing_too_above_threshold
        return request
