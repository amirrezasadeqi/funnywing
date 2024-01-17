from std_msgs.msg import Bool
from pymavlink import mavutil


class rescue_status_to_std_msgs_Bool(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_rescue_status_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        Converts MAVLink_rescue_status_message mavlink message to std_msgs/Bool message.
        """
        rosMsg = Bool()
        rosMsg.data = True if mavutil.mavlink.RESCUE_ENABLED == self._message.rescue_state else False
        return rosMsg
