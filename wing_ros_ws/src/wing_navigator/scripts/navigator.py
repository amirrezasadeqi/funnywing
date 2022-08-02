#!/usr/bin/env python

import rospy
import argparse
# from wing_navigator.srv import *
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from wing_modules.navigator_modules.navigator import navigator

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
  args = parser.parse_args()
  connection_string = args.connect


  # print("Connecting to vehicle on: %s" % connection_string)
  # vehicle = connect(connection_string, wait_ready=True)
  nav_agent = navigator(connection_string)


  # running the goto server
  # simple_goto_server()

  
  rospy.spin() # This may be moved into navigator class for better code base(more extensibility!)

  print("Close Vehicle connection object!")
  # vehicle.close()
  nav_agent.vehicle.close()

