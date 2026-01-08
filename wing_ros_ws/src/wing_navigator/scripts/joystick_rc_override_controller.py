#!/usr/bin/env python3
import rospy
from wing_modules.JoystickRCOverride import JoystickRCOverride

if __name__ == "__main__":
    rospy.init_node("joy_to_rc_override_node", anonymous=True)
    joystick_controller = JoystickRCOverride()
    rospy.spin()
