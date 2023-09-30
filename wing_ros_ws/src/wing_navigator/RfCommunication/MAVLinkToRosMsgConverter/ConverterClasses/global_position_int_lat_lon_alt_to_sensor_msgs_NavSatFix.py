import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header


class global_position_int_lat_lon_alt_to_sensor_msgs_NavSatFix(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_global_position_int_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        convert mavlink global_position_int message to NavSatFix ROS message
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = NavSatFix()
        rosMsg.header = self._getRosMsgHeader()
        # The lat and long are scaled due to low resolution of floats, so descale them.
        rosMsg.latitude = self._message.lat / 1.0e7
        rosMsg.longitude = self._message.lon / 1.0e7
        rosMsg.altitude = self._message.alt / 1000.0  # Height in MSL
        rosMsg.status.status = NavSatStatus.STATUS_SBAS_FIX
        rosMsg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        return rosMsg

    def _getRosMsgHeader(self) -> Header:
        header = Header()
        header.stamp = rospy.Time.from_sec(self._message.time_boot_ms / 1000.0)
        header.frame_id = 'gps'
        return header
