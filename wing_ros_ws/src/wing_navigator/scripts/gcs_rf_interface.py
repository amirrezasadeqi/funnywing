#!/usr/bin/env python3

import rospy
import threading
import os
import time
import argparse
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from wing_modules.navigator_modules.TimerAP import TimerAP


def reader_worker(port, input_msgs):
    '''
        Just reads the input as fast as possible, no data processing to prevent
        data loss.
    '''
    while not rospy.is_shutdown():
        input_msgs.append(port.recv_match(blocking=True, timeout=1.0))


def pub_worker(input_msgs):

    gps_pub = rospy.Publisher(
        "funnywing_gps", NavSatFix, queue_size=1)

    cmd_resp_pub = rospy.Publisher(
        "funnywing_cmd_resp", String, queue_size=1)

    while not rospy.is_shutdown():
        if len(input_msgs) != 0:
            msg = input_msgs.pop(0)
            if (not msg) and (msg.get_type() == "BAD_DATA"):
                continue
            else:
                publisher = gps_pub if msg.get_type() == "GPS_DATA" else cmd_resp_pub
                msg = mav_to_ros_msg(msg)
                publisher.publish(msg)
        else:
            continue


def mav_to_ros_msg(msg):
    '''converts mavlink message into ros message'''
    return msg


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output")
    parser.add_argument("-s", "--system")
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", default=57600)
    args = parser.parse_args()

    rospy.init_node(f"{args.system}_rf_interface")

    os.environ["MAVLINK20"] = "1"
    port = mavutil.mavlink_connection(
        args.serial_port, baud=args.baudrate, dialect="funnywing")

    input_msgs = []

    reader_thread = threading.Thread(
        target=reader_worker, args=(port, input_msgs))
    publisher_thread = threading.Thread(
        target=pub_worker, args=(input_msgs, ))

    reader_thread.start()
    publisher_thread.start()
