import rospy
from mavros_msgs.srv import CommandLong, CommandLongRequest, CommandLongResponse
from pymavlink import mavutil

from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class command_long_job(JobInterface):
    _cmdLongProxy = None

    def __init__(self, message, rfConnection: ConnectionInterface, system, component):
        """

        @type message: MAVLink_command_long_message
        """
        super().__init__(message, rfConnection, system, component)
        if (command_long_job._cmdLongProxy is None) and self._isSystemMavrosSide():
            rospy.wait_for_service("/mavros/cmd/command")
            command_long_job._cmdLongProxy = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        self._request = self._createRequest()
        self._response = CommandLongResponse()
        return

    def _doJob(self):
        if self._isSystemMavrosSide():
            self._response = command_long_job._cmdLongProxy(self._request)
            rospy.loginfo(f"Success: {self._response.success}, Result: {self._response.result}.")
        return

    def _createRequest(self) -> CommandLongRequest:
        request = CommandLongRequest()
        request.broadcast = 0
        request.command = self.getMessage().command
        request.confirmation = self.getMessage().confirmation
        request.param1 = self.getMessage().param1
        request.param2 = self.getMessage().param2
        request.param3 = self.getMessage().param3
        request.param4 = self.getMessage().param4
        request.param5 = self.getMessage().param5
        request.param6 = self.getMessage().param6
        request.param7 = self.getMessage().param7
        return request

    def _isSystemMavrosSide(self):
        """
        @return: This function returns True if RfCommunicationHandler is created in the system connected to the
        autopilot via mavros node. So the wait for mavros service does not block the code.
        """
        return mavutil.mavlink.MAV_TYPE_FIXED_WING == self._system and 1 == self._component
