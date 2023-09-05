#!/usr/bin/env python


import rospy
from ConnectionInterface import ConnectionInterface
from RfConnection import RfConnection
from mavros import mavlink
from mavros_msgs.msg import StatusText
import argparse

if __name__ == "__main__":
    rospy.init_node("RfConnectionTest", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--system")
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", default=9600)
    args = parser.parse_args()

    connection = RfConnection(args.serial_port, args.baudrate)

    count = 0
    while count < 3:
        rosmsg = StatusText()
        rosmsg.text = f"{count} : Hello from {args.system}"
        bits = mavlink.convert_to_bytes(rosmsg)
        mavmsg = connection.getPort().mav.decode(bits)
        connection.write(mavmsg)
        count += 1

    while count > 0:
        inMsg = connection.read()
        if inMsg:
            count -= 1
            print(f"Message: {inMsg}")

    if not count:
        print("Test Passed!")
