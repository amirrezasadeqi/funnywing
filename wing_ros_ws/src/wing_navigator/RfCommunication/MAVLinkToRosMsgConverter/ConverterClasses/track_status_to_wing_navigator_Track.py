from wing_navigator.msg import Track


class track_status_to_wing_navigator_Track(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_track_status_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        Converts MAVLink_track_status_message mavlink message to wing_navigator/Track message.
        """
        rosMsg = Track()
        rosMsg.time_stamp = int(self._message.time_stamp)
        rosMsg.track_id = int(self._message.track_id)
        rosMsg.frame_count = int(self._message.frame_count)
        rosMsg.track_state = int(self._message.track_state)
        rosMsg.top_left_x = float(self._message.top_left_x)
        rosMsg.top_left_y = float(self._message.top_left_y)
        rosMsg.bottom_right_x = float(self._message.bottom_right_x)
        rosMsg.bottom_right_y = float(self._message.bottom_right_y)
        return rosMsg
