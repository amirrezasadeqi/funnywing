#!/usr/bin/env python3


import rospy
from wing_navigator.msg import GLOBAL_POSITION_INT


if __name__ == "__main__":
    rospy.init_node("dummy_target_gps_publisher")
    r = rospy.Rate(4)
    pub = rospy.Publisher("/target_gps_topic",
                          GLOBAL_POSITION_INT, queue_size=1)
    msg = GLOBAL_POSITION_INT()
    msg.gps_data.latitude = 35.41330190
    msg.gps_data.longitude = 51.15932984
    msg.gps_data.altitude = 50
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()
