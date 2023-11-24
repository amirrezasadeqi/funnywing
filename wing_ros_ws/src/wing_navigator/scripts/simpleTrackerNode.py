#!/usr/bin/env python

import sys
import rospy
import argparse
from pymavlink import mavutil

from wing_modules.simpleTracker.SimpleTracker import SimpleTracker
from wing_modules.simpleTracker.CommandSender import CommandSenderLocal, CommandSenderRemote

if __name__ == "__main__":
    rospy.init_node("simpleTrackerNode")

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--system", default="GCS")
    args = parser.parse_args()

    if "GCS" == args.system:
        wingGPSTopic = "/funnywing/globalPosition"
        targetGPSTopic = "/target/globalPosition"
        sysID = mavutil.mavlink.MAV_TYPE_GCS
        compID = 1
        commandSender = CommandSenderRemote(sysID, compID, "/GCS/from")
        local = False
    elif "RPI" == args.system:
        wingGPSTopic = "/mavros/global_position/global"
        targetGPSTopic = "/funnywing/targetGlobalPosition"
        commandSender = CommandSenderLocal()
        local = True
    else:
        rospy.logerr("No CommandSenderInterface Implementation exists for the specified system. Please Implement "
                     "CommandSenderInterface for your system!")
        sys.exit()

    # Tune this based on your plane maneuverability and also set the waypoint radius in the ArduPlane.
    waypointRadius = 120
    commandSender.setWayPointRadius(waypointRadius)
    # To touch the target, virtualTargetOffset must be equal or greater than plane waypoint radius. The optimum offset
    # is waypoint radius plus a little value to be confident about touching the target point.
    simpleTracker = SimpleTracker(commandSender, wingGPSTopic, targetGPSTopic, "/virtualTarget/globalPosition",
                                  virtualTargetOffset=waypointRadius + 5, local=local,
                                  rfConnectionTopic="/mavlink/from")
    # Virtual target is determined around the wing if True and around target if False.
    simpleTracker.useWingForVirtualTargetCenter(True)

    # SimpleTracker class has its own thread for spinning to not block the code using it, so we should prevent the
    # program to exit immediately. TODO: Check for a better way to prevent the code to exit, like keyboard interrupt ...
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
