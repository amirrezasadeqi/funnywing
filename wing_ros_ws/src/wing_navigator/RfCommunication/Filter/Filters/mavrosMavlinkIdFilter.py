import json
import rospy
from pymavlink import mavutil

from RfCommunication.Filter.Interface.FilterInterface import FilterInterface


class mavrosMavlinkIdFilter(FilterInterface):
    def __init__(self, configPath):
        """
        This filter returns True if the message type(mavlink type, e.g. HEARTBEAT) of the mavros_msgs/Mavlink message
        exists in the allowedList AND does not in the blackList of the filter configurations. Note that the allowedList
        overrides the blackList.

        self._mavlink_map is a dictionary that maps the numerical mavlink message id to the class implementing that
        message. It can be found in the python file implementing your desired mavlink dialect(e.g. common dialect
        implementation file). Here, preloaded, since I think(I don't know exactly) it make the performance better.
        """
        self._config = self._readFilterConfig(configPath)
        self._mavlink_map = mavutil.mavlink.mavlink_map
        return

    def msgIsPassed(self, message) -> bool:
        """
        @param message: mavros_msgs/MAVLink ROS message
        @return: Returns True if the message type is in allowedList AND not in blackList. So, blackList overrides the
        allowedList.
        """
        try:
            messageType = self._mavlink_map[message.msgid].msgname
            return (messageType in self._config["allowedList"]) and (messageType not in self._config["blackList"])
        except KeyError:
            rospy.logwarn("The message is not existed in the chosen dialect XML file. Check if it is published "
                          "wrongly or add this message to your custom dialect.")
            return False

    def _readFilterConfig(self, configPath):
        configFile = open(configPath)
        return json.load(configFile)["mavrosMavlinkIdFilter"]
