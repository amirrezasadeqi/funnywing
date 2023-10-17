import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler


class attitude_to_sensor_msgs_Imu(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_attitude_message.
        """
        self._message = message
        self._eulerAngles = [message.roll, message.pitch, message.yaw]
        self._eulerConvention = 'rzyx'
        # Quaternion list format is: [x, y, z, w]
        self._quaternions = [0, 0, 0, 0]
        self._convertEulerToQuaternion()
        return

    def convertToRosMsg(self):
        """
        convert mavlink ATTITUDE message to sensor_msgs/Imu ROS message. ATTITUDE is representing orientation in Euler
        angles and in Z-down, Y-right, X-front, ZYX intrinsic(refers to elemental rotations around rotating coordinates)
        coordinate frame. To convert between Euler angles and quaternions and vice versa, use below link:

        https://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/
        """
        rosMsg = Imu()
        rosMsg.header = self._getRosMsgHeader()
        rosMsg.orientation.x = self._quaternions[0]
        rosMsg.orientation.y = self._quaternions[1]
        rosMsg.orientation.z = self._quaternions[2]
        rosMsg.orientation.w = self._quaternions[3]
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

    def _convertEulerToQuaternion(self):
        self._quaternions = quaternion_from_euler(self._eulerAngles[0], self._eulerAngles[1], self._eulerAngles[2],
                                                  axes=self._eulerConvention)
        return
