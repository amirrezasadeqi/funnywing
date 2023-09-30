import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Header

# Arduplane mode map to convert integer flight modes into Strings mode codes used in the State message
# from https://github.com/mavlink/mavros/blob/78b527b831f30df4e44972b75c07294227e313fb/mavros/src/lib/uas_stringify.cpp#L32
ARDUPLANE_MODE_MAP = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    15: "GUIDED"
}


class heartbeat_to_mavros_msgs_State(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_heartbeat_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        Converts HEARTBEAT mavlink message to mavros_msgs/State message.
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = State()
        rosMsg.header = self._getRosMsgHeader()
        rosMsg.mode = ARDUPLANE_MODE_MAP[self._message.custom_mode]
        rosMsg.system_status = self._message.system_status
        return rosMsg

    def _getRosMsgHeader(self) -> Header:
        header = Header()
        return header
