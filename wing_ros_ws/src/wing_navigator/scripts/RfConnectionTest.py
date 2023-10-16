#!/usr/bin/env python


import rospy
import argparse
from pymavlink import mavutil
from rospkg import RosPack as rospack

from RfCommunication.Job.Factory.JobFactory import JobFactory
from RfCommunication.MavrosPublishManager.MavrosPublishManager import MavrosPublishManager
from RfCommunication.RfCommunicationHandler.RfCommunicationHandler import RfCommunicationHandler
from RfCommunication.RfConnection.RfConnection import RfConnection
from RfCommunication.Filter import Filters

########################################################################
# TODO: Delete below test things.
# For Test
# from pymavlink.dialects.v20.funnywing import mavlink_map
# import pymavlink.dialects.v20.funnywing as funnywing
# from pymavlink import mavutil
# mavlink_map[funnywing.MAVLINK_MSG_ID_HEARTBEAT]
########################################################################
systemAddressMap = {
    "GCS": {
        "systemID": mavutil.mavlink.MAV_TYPE_GCS,
        "componentID": 1
    },

    "RPI": {
        "systemID": mavutil.mavlink.MAV_TYPE_FIXED_WING,
        "componentID": 1  # Determined from SITL.
    }
}

if __name__ == "__main__":
    rospy.init_node("RfConnectionTest", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--system")
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", default=115200)
    parser.add_argument("-d", "--dialect", default="funnywing")
    args = parser.parse_args()

    publisherConfigPath = rospack().get_path("wing_navigator")
    filterConfigPath = rospack().get_path("wing_navigator")

    if "GCS" == args.system:
        publisherConfigPath += "/Configs/MavrosPublishManagerGCSConfigs.json"
        filterConfigPath += "/Configs/FiltersGCSConfigs.json"
    elif "RPI" == args.system:
        publisherConfigPath += "/Configs/MavrosPublishManagerRPIConfigs.json"
        filterConfigPath += "/Configs/FiltersRPIConfigs.json"
    else:
        print("Please Specify a valid configuration file for your system!")

    systemAddress = systemAddressMap[args.system]

    # Creating list of message filters. Adding mavrosInternalMsgFilter first preventing from warning logs about not
    # defined mavlink messages.
    messageFilters = []
    mavrosInternalMsgFilter = Filters.mavrosInternalMsgFilter(filterConfigPath)
    messageFilters.append(mavrosInternalMsgFilter)
    mavrosMavLinkIdFilter = Filters.mavrosMavlinkIdFilter(filterConfigPath)
    messageFilters.append(mavrosMavLinkIdFilter)

    connection = RfConnection(args.serial_port, args.baudrate, systemAddress["systemID"], systemAddress["componentID"],
                              args.dialect)

    mavrosPubMng = MavrosPublishManager(publisherConfigPath, connection)

    # For now the system is not important in the Job we create. later we pass specific IDs for system and component.
    jf = JobFactory(connection, systemAddress["systemID"], systemAddress["componentID"])

    comHandler = RfCommunicationHandler(connection, jf, mavrosPubMng, messageFilters, systemAddress["systemID"],
                                        systemAddress["componentID"],
                                        "/mavlink/from" if "RPI" == args.system else "/GCS/from")
