import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped


class position_target_global_int_vx_vy_vz_to_geometry_msgs_TwistStamped(object):

    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is
        MAVLink_position_target_global_int_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        Converts position_target_global_int (velocity fields) to TwistStamped ROS message
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = TwistStamped()
        rosMsg.header = self._getRosMsgHeader()
        # POSITION_TARGET_GLOBAL_INT message velocities are in m/sec
        rosMsg.twist.linear.x = self._message.vx
        rosMsg.twist.linear.y = self._message.vy
        rosMsg.twist.linear.z = self._message.vz
        return rosMsg

    def _getRosMsgHeader(self) -> Header:
        header = Header()
        header.stamp = rospy.Time.from_sec(self._message.time_boot_ms / 1000.0)
        header.frame_id = 'targetGPS'
        return header
