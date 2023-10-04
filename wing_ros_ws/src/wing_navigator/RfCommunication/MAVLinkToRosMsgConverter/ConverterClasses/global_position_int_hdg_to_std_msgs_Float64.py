from std_msgs.msg import Float64


class global_position_int_hdg_to_std_msgs_Float64(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_global_position_int_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        convert mavlink global_position_int(heading field) message to Float64 ROS message
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = Float64()
        # Vehicle heading in GLOBAL_POSITION_INT is in centi degrees.
        rosMsg.data = self._message.hdg / 100.0
        return rosMsg
