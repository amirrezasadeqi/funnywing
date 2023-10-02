#!/usr/bin/env python

import argparse
import json
import rospy
from rospkg import RosPack as rospack

from RfCommunication.Job.Factory.JobFactory import JobFactory
from RfCommunication.RfCommunicationHandler.RfCommunicationHandler import RfCommunicationHandler
from RfCommunication.RfConnection.RfConnection import RfConnection
from RfCommunication.MavrosPublishManager.MavrosPublishManager import MavrosPublishManager

########################################################################
# TODO: Delete below test things.
# For Test
# from pymavlink.dialects.v20.funnywing import mavlink_map
# import pymavlink.dialects.v20.funnywing as funnywing
# from pymavlink import mavutil
# mavlink_map[funnywing.MAVLINK_MSG_ID_HEARTBEAT]
########################################################################


if __name__ == "__main__":
    rospy.init_node("RfConnectionTest", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--system")
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", default=115200)
    parser.add_argument("-d", "--dialect", default="funnywing")
    args = parser.parse_args()

    config_path = rospack().get_path("wing_navigator")

    if "GCS" == args.system:
        config_path += "/Configs/MavrosPublishManagerGCSConfigs.json"
    elif "RPI" == args.system:
        config_path += "/Configs/MavrosPublishManagerRPIConfigs.json"
    else:
        print("Please Specify a valid configuration file for your system!")

    mavrosPubMng = MavrosPublishManager(config_path)
    connection = RfConnection(args.serial_port, args.baudrate, args.dialect)
    # For now the system is not important in the Job we create. later we pass specific IDs for system and component.
    jf = JobFactory(connection, mavrosPubMng, 0, 0)

    comHandler = RfCommunicationHandler(connection, jf, args.system,
                                        "/mavlink/from" if "RPI" == args.system else "/GCS/from")
