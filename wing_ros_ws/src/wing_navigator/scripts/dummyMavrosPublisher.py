import rospy
import argparse
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v20 import funnywing
from mavros import mavlink

if __name__ == "__main__":
    rospy.init_node("dummyMavrosPublisher")
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--system")
    args = parser.parse_args()
    publisher_name = "/from_" + args.system + "_ros"
    mavPub = rospy.Publisher(publisher_name, Mavlink, queue_size=10)
    rate = rospy.Rate(1)

    sysType = funnywing.MAV_TYPE_GCS if "GCS" == args.system else funnywing.MAV_TYPE_FIXED_WING
    mavMsg = funnywing.MAVLink_heartbeat_message(sysType, 0, 0, 0, 0, 0)
    mavrosMsg = mavlink.convert_to_rosmsg(mavMsg)
    while not rospy.is_shutdown():
        mavPub.publish(mavrosMsg)
        rate.sleep()
