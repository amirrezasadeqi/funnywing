#!/usr/bin/env python

import rospy
import argparse
# test
from std_msgs.msg import String

from wing_navigator.srv import ActiveMode, ArmTakeoff, PreDefMission, SimpleGoto, MissionInOut, WP_list_save, WP_list_upload
# from wing_navigator.srv import *
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from wing_modules.navigator_modules.navigator import copter_navigator, fw_navigator, navigator

# def simple_goto_handler(req):
#   try:
#     print("Going to your desired position")
#     vehicle.mode = VehicleMode("GUIDED")
#     goto_location = LocationGlobalRelative(req.lat, req.lon, req.alt)
#     vehicle.simple_goto(goto_location)
#     resp = SimpleGotoResponse()
#     resp.accepted = True
#     return resp
#   except:
#     resp = SimpleGotoResponse()
#     resp.accepted = False
#     return resp


# def simple_goto_server():
#   s = rospy.Service("/simple_goto", SimpleGoto, simple_goto_handler)
#   print("Ready for goto service")
#   rospy.spin()

if __name__ == '__main__':

    rospy.init_node("navigator")

    parser = argparse.ArgumentParser(description='this is a test program')
    parser.add_argument('--connect', help='Address of vehicle connection.')
    parser.add_argument('--connect_copter',
                        help='Address of target vehicle connection')
    args = parser.parse_args()
    connection_string = args.connect
    copter_connection_string = args.connect_copter

    # This part is experimental
    #######################################################################
    # For more Info goto the navigator class implementation
    list_of_servers_dict = [{"server_name": "simple_goto", "server_data_type": SimpleGoto, "server_handler_type": "simple_goto"},
                            {"server_name": "active_mode", "server_data_type": ActiveMode,
                                "server_handler_type": "active_mode"},
                            {"server_name": "arm_takeoff", "server_data_type": ArmTakeoff,
                                "server_handler_type": "arm_takeoff"},
                            {"server_name": "save_mission", "server_data_type": MissionInOut,
                                "server_handler_type": "save_mission"},
                            {"server_name": "upload_mission", "server_data_type": MissionInOut,
                                "server_handler_type": "upload_mission"},
                            {"server_name": "save_mission_ros", "server_data_type": WP_list_save,
                                "server_handler_type": "save_mission_ros"},
                            {"server_name": "upload_mission_ros", "server_data_type": WP_list_upload,
                                "server_handler_type": "upload_mission_ros"},
                            {"server_name": "upload_predefined_mission", "server_data_type": PreDefMission, "server_handler_type": "upload_predefined_mission"}]
    # {"server_name": "save_mission_ros", "server_data_type": WP_list_save, "server_handler_type": "save_mission"},
    # {"server_name": "upload_mission_ros", "server_data_type": WP_list_upload, "server_handler_type": "upload_mission"}]

    # Adding pub/sub capabillity to navigator agents, e.g. for reading GPS data of the drone
    # TODO: complete this part for publish and subscribe data
    list_of_publisher_dict = [
        {"publisher_name": "gps_publisher", "topic_name": "gps_data",
            "publisher_data_type": String, "rate": 4.0, "queue_size": 1, "pub_handler_type": "gps_pub_handler"}
    ]

    list_of_subscriber_dict = [
    ]

    # nav_agent = navigator("flying_wing", connection_string, list_of_servers_dict)
    fw_nav_agent = fw_navigator(
        "wing", connection_string, list_of_servers_dict, list_of_publisher_dict, list_of_subscriber_dict)
    if copter_connection_string:
        copter_nav_agent = copter_navigator(
            "target", copter_connection_string, list_of_servers_dict, list_of_publisher_dict, list_of_subscriber_dict)

    ########################################################################

    # print("Connecting to vehicle on: %s" % connection_string)
    # vehicle = connect(connection_string, wait_ready=True)
    # nav_agent = navigator(connection_string)

    # running the goto server
    # simple_goto_server()

    rospy.spin()  # This may be moved into navigator class for better code base(more extensibility!)

    # closing the vehicle moved into navigator destructors
    # print("Close Vehicle connection object!")
    # vehicle.close()
    # nav_agent.vehicle.close()
