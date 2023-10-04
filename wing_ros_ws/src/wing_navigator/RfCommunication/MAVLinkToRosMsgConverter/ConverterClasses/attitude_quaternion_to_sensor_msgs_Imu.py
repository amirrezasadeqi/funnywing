import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu


class attitude_quaternion_to_sensor_msgs_Imu(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_attitude_quaternion_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        convert mavlink ATTITUDE_QUATERNION message to sensor_msgs/Imu ROS message. ATTITUDE_QUATERNION is representing
        orientation in quaternion and in Z-down, Y-right, X-front. To convert between quaternions and euler angles and
        vice versa, use below link:
        https://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = Imu()
        rosMsg.header = self._getRosMsgHeader()
        rosMsg.orientation.w = self._message.q1
        rosMsg.orientation.x = self._message.q2
        rosMsg.orientation.y = self._message.q3
        rosMsg.orientation.z = self._message.q4
        # Angular Velocities are in [rad/s]
        rosMsg.angular_velocity.x = self._message.rollspeed
        rosMsg.angular_velocity.y = self._message.pitchspeed
        rosMsg.angular_velocity.z = self._message.yawspeed
        return rosMsg

    def _getRosMsgHeader(self) -> Header:
        header = Header()
        header.stamp = rospy.Time.from_sec(self._message.time_boot_ms / 1000.0)
        header.frame_id = 'imu'
        return header
