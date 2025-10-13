import rospy
from std_msgs.msg import Header
from mavros_msgs.msg import VFR_HUD

class vfr_hud_to_std_msgs_Vfr_Hud(object):
    def __init__(self, message):
        self._message = message
        return
    
    def convertToRosMsg(self):
        rosMsg = VFR_HUD()
        rosMsg.header = self._getRosMsgHeader()
        
        rosMsg.airspeed = self._message.airSpeed
        rosMsg.groundspeed = self._message.groundSpeed
        rosMsg.throttle = float(self._message.throttle)
        
    def _getRosMsgHeader(self) -> Header:
        header = Header()
        header.stamp = rospy.Time.from_sec(self._message.time_boot_ms / 1000.0)
        header.frame_id = 'vfr_hud'
        return header