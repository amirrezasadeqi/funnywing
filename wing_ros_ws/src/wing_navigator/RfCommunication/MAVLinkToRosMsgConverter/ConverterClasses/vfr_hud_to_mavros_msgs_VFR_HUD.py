from std_msgs.msg import Header
from mavros_msgs.msg import VFR_HUD

class vfr_hud_to_mavros_msgs_VFR_HUD(object):
    def __init__(self, message):
        self._message = message
        return
    
    def convertToRosMsg(self):
        rosMsg = VFR_HUD()
        
        rosMsg.airspeed = float(self._message.airspeed)
        rosMsg.groundspeed = float(self._message.groundspeed)
        rosMsg.throttle = float(self._message.throttle)
        rosMsg.heading = int(self._message.heading)
        rosMsg.altitude = float(self._message.alt)
        rosMsg.climb = float(self._message.climb)
        
        return rosMsg
        

