#!/usr/bin/env python

#######################################################################################################################
#   TODO: This is a file to write try/error things and so on. Don't leave any valuable code here, since it will be
#    deleted and Transfer valuable codes to major files of the Project.
######################################################################################################################

import rospy
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from mavros import mavlink
from mavros_msgs.srv import CommandLong, CommandLongRequest, CommandInt, CommandIntRequest

if "__main__" == __name__:
    rospy.init_node("playGroundCodes", anonymous=True)
    armState = 1
    # mavMsg = mavutil.mavlink.MAVLink_command_long_message(mavutil.mavlink.MAV_TYPE_FIXED_WING, 1,
    #                                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    #                                                       armState, 0, 0, 0, 0, 0, 0)

    # mavMsg = mavutil.mavlink.MAVLink_command_long_message(mavutil.mavlink.MAV_TYPE_FIXED_WING, 1,
    #                                                       mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #                                                       1, 15, 0, 0, 0, 0, 0)
    #
    # rospy.wait_for_service("/mavros/cmd/command")
    # comProxy = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
    # req = CommandLongRequest()
    # req.broadcast = 0
    # req.command = mavMsg.command
    # req.confirmation = mavMsg.confirmation
    # req.param1 = mavMsg.param1
    # req.param2 = mavMsg.param2
    # req.param3 = mavMsg.param3
    # req.param4 = mavMsg.param4
    # req.param5 = mavMsg.param5
    # req.param6 = mavMsg.param6
    # req.param7 = mavMsg.param7
    # resp = comProxy(req)
    #
    # rospy.loginfo(resp.success)

    lat = int(35.41319230 * 1e7)
    lon = int(51.16245796 * 1e7)
    alt = 20.0
    mavMsg = mavutil.mavlink.MAVLink_command_int_message(mavutil.mavlink.MAV_TYPE_FIXED_WING, 1,
                                                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                         mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1,
                                                         mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 120,
                                                         0, lat, lon, alt)

    # rospy.wait_for_service("/mavros/cmd/command_int")
    # intcomProxy = rospy.ServiceProxy("/mavros/cmd/command_int", CommandInt)
    # req = CommandIntRequest()
    # req.broadcast = 0
    # req.frame = mavMsg.frame
    # req.command = mavMsg.command
    # req.current = mavMsg.current
    # req.autocontinue = mavMsg.autocontinue
    # req.param1 = mavMsg.param1
    # req.param2 = mavMsg.param2
    # req.param3 = mavMsg.param3
    # req.param4 = mavMsg.param4
    # req.x = mavMsg.x
    # req.y = mavMsg.y
    # req.z = mavMsg.z
    # resp = intcomProxy(req)
    #
    # rospy.loginfo(resp.success)

    # For sending data to autopilot using publisher you should publish it in a loop and a single publish
    # does not work.
    toMavlinkPub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=1)
    link = mavutil.mavlink.MAVLink("")
    mavMsg.pack(link)
    rosMsg = mavlink.convert_to_rosmsg(mavMsg)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        toMavlinkPub.publish(rosMsg)
        r.sleep()
