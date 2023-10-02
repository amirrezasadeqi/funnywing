#!/usr/bin/env python
import rospy
import argparse
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from mavros import mavlink

if __name__ == "__main__":
    rospy.init_node("dummyMavrosPublisher")
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--system")
    parser.add_argument("-t", "--topic_to_publish")
    args = parser.parse_args()
    publisher_name = args.topic_to_publish
    mavPub = rospy.Publisher(publisher_name, Mavlink, queue_size=10)
    rate = rospy.Rate(1)

    sysType = mavutil.mavlink.MAV_TYPE_GCS if "GCS" == args.system else mavutil.mavlink.MAV_TYPE_FIXED_WING
    mavMsg = mavutil.mavlink.MAVLink_heartbeat_message(sysType, 0, 0, 0, 0, 0)
    mavMsg.pack(mavutil.mavlink.MAVLink(''))
    mavrosMsg = mavlink.convert_to_rosmsg(mavMsg)
    while not rospy.is_shutdown():
        mavPub.publish(mavrosMsg)
        rate.sleep()
