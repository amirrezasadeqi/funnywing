from std_msgs.msg import Float64


class global_position_int_relative_alt_to_std_msgs_Float64(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_global_position_int_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        convert mavlink global_position_int(relative_alt field) message to Float64 ROS message
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = Float64()
        # Vehicle relative_alt in GLOBAL_POSITION_INT is in millimeter.
        rosMsg.data = self._message.relative_alt / 1000.0
        return rosMsg
