import rospy
from pymavlink import mavutil
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class rescue_set_state_job(JobInterface):
    _rescueSetStateProxy = None

    def __init__(self, message, rfConnection: ConnectionInterface, system, component):
        """

        @type message: MAVLink_rescue_set_state_message
        """
        super().__init__(message, rfConnection, system, component)
        if rescue_set_state_job._rescueSetStateProxy is None:
            rospy.wait_for_service("/funnywing/setRescueState")
            rescue_set_state_job._rescueSetStateProxy = rospy.ServiceProxy("/funnywing/setRescueState", SetBool)
        self._request = self._createRequest()
        self._response = SetBoolResponse()
        return

    def _doJob(self):
        self._response = rescue_set_state_job._rescueSetStateProxy(self._request)
        rospy.loginfo(f"Success: {self._response.success}")
        return

    def _createRequest(self) -> SetBoolRequest:
        request = SetBoolRequest()
        request.data = True if mavutil.mavlink.RESCUE_ENABLED == self.getMessage().rescue_state else False
        return request
