#!/usr/bin/env python3

import rospy
import argparse
from dronekit import connect
from wing_navigator.msg import GLOBAL_POSITION_INT

if __name__ == "__main__":
    rospy.init_node("target_data_reciever")
    r = rospy.Rate(5)

    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--serial_port", default="/dev/ttyACM0")
    parser.add_argument("-b", "--baudrate", default=9600)
    args = parser.parse_args()

    vehicle = connect(args.serial_port, baud=args.baudrate, wait_ready=True)

    gps_publisher = rospy.Publisher("/target_gps_topic", GLOBAL_POSITION_INT, queue_size=1)
    msg = GLOBAL_POSITION_INT()

    while not rospy.is_shutdown():
        tg_loc = vehicle.location.global_frame
        msg.gps_data.latitude = tg_loc.lat
        msg.gps_data.longitude = tg_loc.lon
        msg.gps_data.altitude = tg_loc.alt
        gps_publisher.publish(msg)
        r.sleep()

    vehicle.close()
