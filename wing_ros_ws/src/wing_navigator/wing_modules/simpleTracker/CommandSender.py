from abc import ABC, abstractmethod

import rospy
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from mavros_msgs.srv import CommandInt, CommandIntRequest
from pymavlink import mavutil


class CommandSenderInterface(ABC):

    def __init__(self):
        """
        @param _wayPointRadius: The radius that the plane circles around the waypoint. This Should be tuned for
        specific plane.
        """
        self._wayPointRadius = 120
        return

    @abstractmethod
    def sendCommand(self, commandGlobalPos):
        pass

    def setWayPointRadius(self, radius):
        self._wayPointRadius = radius
        return

    def _constructMavMsg(self, commandGlobalPos):
        # Scaling lat, lon to use them with MAVLink_command_int_message.
        lat = int(commandGlobalPos[0] * 1e7)
        lon = int(commandGlobalPos[1] * 1e7)
        alt = commandGlobalPos[2]
        # Using MAV_FRAME_GLOBAL_INT to not being worry about the home altitude for sending go to command.
        # Use mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE instead of 0 in param2 of the command to
        # automatically change the mode to GUIDED, otherwise you should explicitly change the mode to GUIDED and then
        # the commands will work.
        mavMsg = mavutil.mavlink.MAVLink_command_int_message(0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                                                             mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1, 0,
                                                             self._wayPointRadius, 0, lat, lon, alt)
        return mavMsg


class CommandSenderLocal(CommandSenderInterface):
    """
    This class is used when the simpleTrackerNode is running locally in RPI.
    """

    def __init__(self):
        super().__init__()
        rospy.wait_for_service("/mavros/cmd/command_int")
        self._comIntProxy = rospy.ServiceProxy("/mavros/cmd/command_int", CommandInt)
        return

    def sendCommand(self, commandGlobalPos):
        request = self._constructRequest(commandGlobalPos)
        rospy.loginfo(f"Go to Response, Success: {self._comIntProxy(request).success}")
        return

    def _constructRequest(self, commandGlobalPos):
        mavMsg = self._constructMavMsg(commandGlobalPos)
        request = CommandIntRequest()
        request.broadcast = 0
        request.frame = mavMsg.frame
        request.command = mavMsg.command
        request.current = mavMsg.current
        request.autocontinue = mavMsg.autocontinue
        request.param1 = mavMsg.param1
        request.param2 = mavMsg.param2
        request.param3 = mavMsg.param3
        request.param4 = mavMsg.param4
        request.x = mavMsg.x
        request.y = mavMsg.y
        request.z = mavMsg.z
        return request


class CommandSenderRemote(CommandSenderInterface):
    """
    This class is used when the simpleTrackerNode is running remotely on the GCS.
    """

    def __init__(self, systemID, componentID, toRfConnectionTopic):
        super().__init__()
        self._protocolObj = mavutil.mavlink.MAVLink('', systemID, componentID)
        self._toRfConnectionPublisher = rospy.Publisher(toRfConnectionTopic, Mavlink, queue_size=10)
        return

    def sendCommand(self, commandGlobalPos):
        mavMsg = self._constructMavMsg(commandGlobalPos)
        rosMsg = self._convertToRosMsg(mavMsg)
        self._toRfConnectionPublisher.publish(rosMsg)
        return

    def _convertToRosMsg(self, mavMsg):
        mavMsg.pack(self._protocolObj)
        return mavlink.convert_to_rosmsg(mavMsg)
