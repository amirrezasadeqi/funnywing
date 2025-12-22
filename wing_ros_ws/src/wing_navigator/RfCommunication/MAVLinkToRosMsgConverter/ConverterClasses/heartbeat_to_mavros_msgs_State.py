import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Header
import time

# MAVLink mode flags (from MAV_MODE_FLAG enum)
MAV_MODE_FLAG_SAFETY_ARMED = 0x80            # (1 << 7)
MAV_MODE_FLAG_GUIDED_ENABLED = 0x20          # (1 << 5)
MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 0x40    # (1 << 6)

# ArduPlane flight mode mapping
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
    """
    Converts MAVLink HEARTBEAT messages to mavros_msgs/State ROS messages,
    with connection timeout detection and ArduPlane GUIDED fix.
    """

    _last_heartbeat_time = 0.0
    _connection_timeout = 2.0 

    def __init__(self, message=None):
        self._message = message

    def convertToRosMsg(self):
        rosMsg = State()
        rosMsg.header = self._getRosMsgHeader()
        now = time.time()

        if self._message is not None:
            self.__class__._last_heartbeat_time = now
            connected = True
        else:
            connected = (now - self.__class__._last_heartbeat_time) < self._connection_timeout

        rosMsg.connected = connected

        if connected and self._message is not None:
            base_mode = self._message.base_mode
            custom_mode = self._message.custom_mode

            rosMsg.armed = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            rosMsg.manual_input = bool(base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)

            rosMsg.guided = bool(base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) or (custom_mode == 15)

            rosMsg.mode = ARDUPLANE_MODE_MAP.get(custom_mode, str(custom_mode))
            rosMsg.system_status = self._message.system_status

        else:
            rosMsg.armed = False
            rosMsg.guided = False
            rosMsg.manual_input = False
            rosMsg.mode = "DISCONNECTED"
            rosMsg.system_status = 0

        return rosMsg

    def _getRosMsgHeader(self) -> Header:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"
        return header

