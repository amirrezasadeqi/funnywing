#!/usr/bin/env python3
import rospy
from pymavlink import mavutil
from sensor_msgs.msg import Joy
from mavros_msgs.msg import Mavlink
from mavros import mavlink

class JoystickRCOverride:
    def __init__(self):
        self._rc_pub = rospy.Publisher("/GCS/from", Mavlink, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        rospy.loginfo("Joy → RC Override initialized.")

        self.last_rc = {"roll": 1500, "pitch": 1500, "throttle": 1500, "yaw": 1500}

        self._current_throttle = 1500

        self._protocolObj = mavutil.mavlink.MAVLink('', mavutil.mavlink.MAV_TYPE_GCS, 1)

        rospy.Timer(rospy.Duration(0.05), self.publish_override) 

    def joy_callback(self, msg: Joy):
        roll_axis = msg.axes[0]
        pitch_axis = msg.axes[1]
        throttle_axis = msg.axes[3]
        yaw_axis = msg.axes[2]

        DEADZONE = 0.05

        self.last_rc["roll"]  = int(1500 - roll_axis  * 500) if abs(roll_axis)  > DEADZONE else 1500
        self.last_rc["pitch"] = int(1500 - pitch_axis * 500) if abs(pitch_axis) > DEADZONE else 1500
        self.last_rc["yaw"]   = int(1500 + yaw_axis   * 500) if abs(yaw_axis)   > DEADZONE else 1500

        THROTTLE_DEADZONE = 0.15
        if abs(throttle_axis) > THROTTLE_DEADZONE:
            new_val = int(1000 + (throttle_axis + 1.0) * 500)  
            self._current_throttle = max(1000, min(2000, new_val))

        self.last_rc["throttle"] = self._current_throttle

    def publish_override(self, _event):
        roll = self.last_rc["roll"]
        pitch = self.last_rc["pitch"]
        throttle = self.last_rc["throttle"]
        yaw = self.last_rc["yaw"]

        rospy.loginfo_throttle(1, f"RC => Roll:{roll} Pitch:{pitch} Throttle:{throttle} Yaw:{yaw}")

        mavMsg = mavutil.mavlink.MAVLink_rc_channels_override_message(
            mavutil.mavlink.MAV_TYPE_FIXED_WING, 1,  
            roll, pitch, throttle, yaw, 0, 0, 0, 0
        )

        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._rc_pub.publish(rosMsg)
