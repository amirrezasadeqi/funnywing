import rospy
from mavros_msgs.srv import CommandInt, CommandIntRequest, CommandIntResponse
from pymavlink import mavutil

from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class command_int_job(JobInterface):
    _comIntProxy = None

    def __init__(self, message, rfConnection: ConnectionInterface, system, component):
        """

        @type message: MAVLink_command_int_message
        """
        super().__init__(message, rfConnection, system, component)
        if (command_int_job._comIntProxy is None) and self._isSystemMavrosSide():
            rospy.wait_for_service("/mavros/cmd/command_int")
            command_int_job._comIntProxy = rospy.ServiceProxy("/mavros/cmd/command_int", CommandInt)
        self._request = self._createRequest()
        self._response = CommandIntResponse()
        return

    def _doJob(self):
        if self._isSystemMavrosSide():
            self._response = command_int_job._comIntProxy(self._request)
            rospy.loginfo(f"Success: {self._response.success}.")
        return

    def _createRequest(self) -> CommandIntRequest:
        request = CommandIntRequest()
        request.broadcast = 0
        request.frame = self.getMessage().frame
        request.command = self.getMessage().command
        request.current = self.getMessage().current
        request.autocontinue = self.getMessage().autocontinue
        request.param1 = self.getMessage().param1
        request.param2 = self.getMessage().param2
        request.param3 = self.getMessage().param3
        request.param4 = self.getMessage().param4
        request.x = self.getMessage().x
        request.y = self.getMessage().y
        request.z = self.getMessage().z
        return request

    def _isSystemMavrosSide(self):
        """
        @return: This function returns True if RfCommunicationHandler is created in the system connected to the
        autopilot via mavros node. So the wait for mavros service does not block the code.
        """
        return mavutil.mavlink.MAV_TYPE_FIXED_WING == self._system and 1 == self._component
