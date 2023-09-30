from mavros import mavlink
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


class all_message_types_to_mavros_msgs_Mavlink(object):
    def __init__(self, message, rfConnection: ConnectionInterface):
        """
        Conversion to mavros_msgs/Mavlink message needs the mav field of the mavlink port. so we pass rfConnection
        Here.

        @param message: MAVLink_<message type> mavlink message. Here the message can be any mavlink message type.
        """
        self._message = message
        self._rfConnection = rfConnection
        return

    def convertToRosMsg(self):
        """
        Converts any mavlink message types to mavros_msgs/Mavlink ROS message.
        """
        self._message.pack(self._rfConnection.getPort().mav)
        rosMsg = mavlink.convert_to_rosmsg(self._message)
        return rosMsg
