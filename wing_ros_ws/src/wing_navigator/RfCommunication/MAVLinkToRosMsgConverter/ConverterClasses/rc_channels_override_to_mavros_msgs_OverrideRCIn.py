from mavros_msgs.msg import OverrideRCIn

class rc_channels_override_to_mavros_msgs_OverrideRCIn(object):
    def __init__(self, message):
        """
        @param message: MAVLink_RC_CHANNELS_OVERRIDE_message
        """
        self._message = message
        self._channels = self._extract_channels()

    def convertToRosMsg(self):
        """
        Convert MAVLink RC_CHANNELS_OVERRIDE message to mavros_msgs/OverrideRCIn ROS message.
        """
        rosMsg = OverrideRCIn()
        rosMsg.channels = self._channels
        
        return rosMsg

    def _extract_channels(self):
        """
        Extract up to 18 RC channel values from MAVLink message.
        If a channel field doesn't exist, it will be set to 0.
        """
        channels = []
        for i in range(1, 19): 
            attr_name = f'chan{i}_raw'
            if hasattr(self._message, attr_name):
                channels.append(getattr(self._message, attr_name))
            else:
                channels.append(0)
        return channels

    