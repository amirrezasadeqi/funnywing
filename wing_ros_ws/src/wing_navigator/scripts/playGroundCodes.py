#!/usr/bin/env python

#######################################################################################################################
#   TODO: This is a file to write try/error things and so on. Don't leave any valuable code here, since it will be
#    deleted and Transfer valuable codes to major files of the Project.
######################################################################################################################

import rospy
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from mavros import mavlink


def mavlinkSubsCallback(msg):
    # protocolObject = mavutil.mavlink.MAVLink('')
    # mavMsg = protocolObject.decode(mavlink.convert_to_bytes(msg))
    # # if "HEARTBEAT" == mavMsg.get_type():
    # if mavMsg.get_header().srcSystem in (0, 255):
    #     rospy.loginfo(mavMsg)
    # # rospy.loginfo(mavMsg.get_header().srcSystem)
    rospy.loginfo(f"system id: {msg.sysid}, component id: {msg.compid}")
    return


if "__main__" == __name__:
    rospy.init_node("playGroundCodesNode")
    sub = rospy.Subscriber("/mavlink/from", Mavlink, callback=mavlinkSubsCallback, queue_size=10)
    # rospy.loginfo(f"Number of publishers to /mavlink/from is {sub.get_num_connections()}")
    rospy.spin()
