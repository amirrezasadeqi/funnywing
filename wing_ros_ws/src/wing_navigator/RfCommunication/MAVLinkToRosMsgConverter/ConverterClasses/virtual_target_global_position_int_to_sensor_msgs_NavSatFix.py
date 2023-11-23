import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header


class virtual_target_global_position_int_to_sensor_msgs_NavSatFix(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is
        MAVLink_virtual_target_global_position_int_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        convert mavlink VIRTUAL_TARGET_GLOBAL_POSITION_INT message to NavSatFix ROS message
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = NavSatFix()
        rosMsg.header = self._getRosMsgHeader()
        # The lat and long are scaled due to low resolution of floats, so descale them.
        rosMsg.latitude = self._message.lat_int / 1.0e7
        rosMsg.longitude = self._message.lon_int / 1.0e7
        rosMsg.altitude = self._message.alt  # Height in meters and MSL
        rosMsg.status.status = NavSatStatus.STATUS_SBAS_FIX
        rosMsg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        return rosMsg

    def _getRosMsgHeader(self) -> Header:
        header = Header()
        header.stamp = rospy.Time.from_sec(self._message.time_boot_ms / 1000.0)
        header.frame_id = 'virtualTargetGPS'
        return header
