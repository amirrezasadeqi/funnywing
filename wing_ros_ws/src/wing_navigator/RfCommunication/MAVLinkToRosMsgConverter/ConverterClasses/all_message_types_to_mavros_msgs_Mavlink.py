import rospy
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
        try:
            self._message.pack(self._rfConnection.getPort().mav)
        except Exception as e:
            # Received messages containing strings can't be converted or resent and based on a question in stackoverflow
            # this is an issue in pymavlink. so we need to convert string fields to bytestrings(I think!). This bug was
            # causing conversion error when we were converting PARAM_SET messages to ros messages for publishing. The
            # reference for this solve is at the below link:
            # https://stackoverflow.com/a/69289595/10243689
            if self._message.get_type() in ('PARAM_SET', 'PARAM_VALUE', 'PARAM_REQUEST_READ'):
                if type(self._message.param_id) == str:
                    self._message.param_id = self._message.param_id.encode()
            elif self._message.get_type() == 'STATUSTEXT':
                if type(self._message.text) == str:
                    self._message.text = self._message.text.encode()
            else:
                rospy.logwarn(e)

        rosMsg = mavlink.convert_to_rosmsg(self._message)
        return rosMsg
