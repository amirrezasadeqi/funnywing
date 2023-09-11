#!/usr/bin/env python


import argparse
import rospy
from pymavlink import mavutil
from RfConnection import RfConnection


def sendMessageCb(event=None):
    global connection
    msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_FIXED_WING, 0, 0, 0, 0, 0)
    connection.write(msg)
    return


def readMessageCb(event=None):
    global connection
    inMsg = connection.read()
    if inMsg:
        print(f"{inMsg}")
    return


if __name__ == "__main__":
    rospy.init_node("RfConnectionTest", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--system")
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", default=9600)
    args = parser.parse_args()

    global connection
    connection = RfConnection(args.serial_port, args.baudrate)

    rospy.Timer(rospy.Duration(0, 1e8), callback=sendMessageCb)
    rospy.Timer(rospy.Duration(0, 1e8), callback=readMessageCb)

    rospy.spin()
