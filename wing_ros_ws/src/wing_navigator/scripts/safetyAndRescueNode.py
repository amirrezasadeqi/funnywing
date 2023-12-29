#!/usr/bin/env python

"""
This code will be run on the Raspberry Pi. So we can access flight data using mavros nodes and topics.
"""

import rospy
import threading
from mavros_msgs.msg import HomePosition, RCIn
from mavros_msgs.srv import SetMode, SetModeRequest
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import numpy as np
import pymap3d

from wing_modules.EllipsoidMSLConversion import EllipsoidMSLConversion


class safetyAndRecueClass(object):
    def __init__(self, altThreshold=15, distanceToHomeThreshold=2000):
        self._altThreshold = altThreshold
        self._distanceToHomeThreshold = distanceToHomeThreshold
        self._ellipsoidMSLConverter = EllipsoidMSLConversion()
        rospy.wait_for_service('/mavros/set_mode')
        self._setModeProxy = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self._homePosSubscriber = rospy.Subscriber("/mavros/home_position/home", HomePosition, self._getHomePosition)
        self._wingPosSubscriber = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._getWingPosition)
        self._wingRelAltSubscriber = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self._getWingRelAlt)
        self._wingRcInSubscriber = rospy.Subscriber("/mavros/rc/in", RCIn, self._getWingRcIn)
        self._rescueThread = threading.Thread(target=self._rescueThreadWorker)
        self._rescueThread.start()
        return

    def _getHomePosition(self, msg: HomePosition):
        self._homePosition = [msg.geo.latitude, msg.geo.longitude, msg.geo.altitude]
        rospy.loginfo(self._homePosition)
        return

    def _getWingPosition(self, msg: NavSatFix):
        self._wingPosition = [msg.latitude, msg.longitude, msg.altitude]
        rospy.loginfo(self._wingPosition)
        return

    def _getWingRelAlt(self, msg: Float64):
        self._wingRelAlt = msg.data
        rospy.loginfo(self._wingRelAlt)
        return

    def _getWingRcIn(self, msg: RCIn):
        self._rcIn = msg.channels
        rospy.loginfo(self._rcIn)
        return

    def _distanceToHome(self):
        homePosWgs = self._ellipsoidMSLConverter.mslToEllipsoid(self._homePosition)
        wingPosWgs = self._ellipsoidMSLConverter.mslToEllipsoid(self._wingPosition)

        homeLocalPos = pymap3d.geodetic2ecef(homePosWgs[0], homePosWgs[1], homePosWgs[2])
        wingLocalPos = pymap3d.geodetic2ecef(wingPosWgs[0], wingPosWgs[1], wingPosWgs[2])

        diff_vec = np.array(homeLocalPos) - np.array(wingLocalPos)
        distanceToHome = np.linalg.norm(diff_vec)
        return distanceToHome

    def _sendChangeModeRequest(self, mode):
        request = SetModeRequest(mode=mode)
        rospy.loginfo(f"Change mode to {mode}, Success: {self._setModeProxy(request).mode_sent}")
        return

    def _rescueThreadWorker(self):
        # TODO: you can also add rate for this loop to prevent from waisting cpu power.
        while not rospy.is_shutdown():
            # TODO: check this condition in field
            if 1500 < self._rcIn[5]:
                rospy.logerr("Rescue and Safety checks are disabled!")
                continue
            elif (self._homePosition is None) or (self._wingPosition is None):
                rospy.logerr("Wing and Home position are still None!")
                continue
            elif self._wingRelAlt < self._altThreshold:
                # change mode loiter or circle
                self._sendChangeModeRequest("LOITER")
            elif self._distanceToHome() > self._distanceToHomeThreshold:
                # change mode to loiter or circle
                self._sendChangeModeRequest("LOITER")
            else:
                continue
        return


if __name__ == "__main__":
    rospy.init_node("safetyAndRescueNode", anonymous=True)
    safetyAndRescueObj = safetyAndRecueClass(altThreshold=15, distanceToHomeThreshold=2000)
    rospy.spin()
