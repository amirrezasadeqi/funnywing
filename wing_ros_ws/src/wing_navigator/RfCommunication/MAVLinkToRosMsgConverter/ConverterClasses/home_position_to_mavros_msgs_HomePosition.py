import rospy
from std_msgs.msg import Header
from mavros_msgs.msg import HomePosition


class home_position_to_mavros_msgs_HomePosition(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_home_position_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        Converts mavlink HOME_POSITION message to mavros_msgs/HomePosition ROS message.
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = HomePosition()
        rosMsg.header = self._getRosMsgHeader()
        rosMsg.geo.latitude = self._message.latitude / 1.0e7
        rosMsg.geo.longitude = self._message.longitude / 1.0e7
        rosMsg.geo.altitude = self._message.altitude / 1000.0  # Height in MSL
        return rosMsg

    def _getRosMsgHeader(self) -> Header:
        header = Header()
        # HOME_POSITION message time stamp is in micro seconds.
        header.stamp = rospy.Time.from_sec(self._message.time_usec / 1.0e6)
        header.frame_id = 'homeGPS'
        return header
