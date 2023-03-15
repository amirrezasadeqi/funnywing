#!/usr/bin/env python3


import rospy
from wing_navigator.msg import GLOBAL_POSITION_INT


if __name__ == "__main__":
    rospy.init_node("dummy_target_gps_publisher")
    r = rospy.Rate(4)
    pub = rospy.Publisher("/target_gps_topic",
                          GLOBAL_POSITION_INT, queue_size=1)
    step = 0.000040
    counter = 0
    msg = GLOBAL_POSITION_INT()
    while not rospy.is_shutdown():
        msg.gps_data.latitude = 35.74743520638767 + counter * step
        msg.gps_data.longitude = 51.60374888014409
        msg.gps_data.altitude = 1200
        pub.publish(msg)
        if counter <= 50:
            counter += 1
        else:
            counter = 0
        r.sleep()
