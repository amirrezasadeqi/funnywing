import rospy
from std_msgs.msg import Header
from mavros_msgs.msg import ManualControl

class manual_control_to_mavros_msgs_ManualControl(object):
    def __init__(self, message):
        """
        @param message: MAVLink MANUAL_CONTROL message
        """
        self._message = message

    def convertToRosMsg(self):
        """
        Convert mavlink MANUAL_CONTROL message to mavros_msgs/ManualControl Ros message
        """
        msg = ManualControl()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "manual_control"
        
        msg.x = float(self._message.x) / 1000.0
        msg.y = float(self._message.y) / 1000.0
        msg.z = float(self._message.z) / 1000.0
        msg.r = float(self._message.r) / 1000.0
        
        msg.buttons = self._message.buttons
        return msg

