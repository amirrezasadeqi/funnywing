#!/usr/bin/env python


import argparse
import rospy
from pymavlink import mavutil

from RfCommunication.Job.Factory.JobFactory import JobFactory
from RfCommunication.RfCommunicationHandler.RfCommunicationHandler import RfCommunicationHandler
from RfCommunication.RfConnection.RfConnection import RfConnection


########################################################################
# TODO: Delete below test things.
# For Test
# from pymavlink.dialects.v20.funnywing import mavlink_map
# import pymavlink.dialects.v20.funnywing as funnywing
# from pymavlink import mavutil
# mavlink_map[funnywing.MAVLINK_MSG_ID_HEARTBEAT]
########################################################################


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

    jf = JobFactory()
    connection = RfConnection(args.serial_port, args.baudrate)
    comHandler = RfCommunicationHandler(connection, jf, args.system)
