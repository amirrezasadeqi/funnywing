import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Joy

class manual_control_to_sensor_msgs_Joy(object):
    def __init__(self, message):
        """
        @param message: MAVLink MANUAL_CONTROL message
        """
        
        self._message = message
    def convertToRosMsg(self):
        """
        Convert mavlink MANUAL_CONTROL message to sensor_msgs/Joy Ros message
        """
        
        joy = Joy()
        joy.header = Header()
        joy.header.stamp = rospy.Time.now()
        
        joy.axes = [
            self._message.x / 1000.0,
            self._message.y / 1000.0,
            self._message.z / 1000.0,
            self._message.r / 1000.0
        ]
        
        buttons = []
        for i in range(16):
            pressed = 1 if (self._message.buttons & (1 << i)) else 0
            buttons.append(pressed)
        joy.buttons = buttons
        
        return joy
