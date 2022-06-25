#!/usr/bin/env python

# include headers and libraries
import rospy
from wing_navigator.srv import *
import sys

def simple_goto_client(req):
  rospy.wait_for_service("/simple_goto")
  try:
    simple_goto_service = rospy.ServiceProxy("/simple_goto", SimpleGoto)
    response = simple_goto_service(req)
    return response.accepted
  except rospy.ServiceException as e:
    print("Service Call Failed: %s"%e)


def usage():
  return "%s [lat lon alt]"%sys.argv[0]


if __name__ == '__main__':
  if len(sys.argv) == 4:
    req = SimpleGotoRequest()
    req.lat = float(sys.argv[1])
    req.lon = float(sys.argv[2])
    req.alt = float(sys.argv[3])
  else:
    print(usage())
    sys.exit(1)

  print("Requesting to for Simple goto service to point (lat:%s, lon:%s, alt:%s)"%(req.lat, req.lon, req.alt))
  print("Request Result: %s"%simple_goto_client(req))
