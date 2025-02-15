#!/usr/bin/env python

#######################################################################################################################
#   TODO: This is a file to write try/error things and so on. Don't leave any valuable code here, since it will be
#    deleted and Transfer valuable codes to major files of the Project.
######################################################################################################################

import rospy
from wing_navigator.srv import GetDouble


def main():
    # this is a test for checking if the zoom can be quickly got by the calling services. I mean to check
    # if the service is fast enough or we should use publishers and subscribers instead.
    r = rospy.Rate(200)
    rospy.wait_for_service("/funnywing/camera/get_camera_zoom")
    proxy = rospy.ServiceProxy("/funnywing/camera/get_camera_zoom", GetDouble)
    while not rospy.is_shutdown():
        try:
            response = proxy()
            print("Response: ", response)
        except rospy.ServiceException as e:
            print("Service call failed: ", e)
        r.sleep()
    return


if "__main__" == __name__:
    rospy.init_node("playGroundCodes", anonymous=True)
    main()
