#!/usr/bin/env python

"""
This code will be run on the Raspberry Pi. So we can access flight data using mavros nodes and topics.
"""

import threading
from enum import Enum

import numpy as np
import pymap3d
import rospy
from mavros import mavlink
from mavros_msgs.msg import HomePosition, RCIn, Waypoint, CommandCode, Mavlink
from mavros_msgs.srv import SetMode, SetModeRequest, WaypointClear, WaypointClearRequest, WaypointPush, \
    WaypointPushRequest
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from wing_modules.EllipsoidMSLConversion import EllipsoidMSLConversion


class rescueState(Enum):
    DISABLE = 0
    ENABLE = 1


class safetyAndRecueClass(object):
    def __init__(self, rcSafetyChannel=7, altThreshold=15, distanceToHomeThreshold=2000, rescueRelAlt=40):
        self._rcSafetyChannelIndex = rcSafetyChannel - 1
        self._altThreshold = altThreshold
        self._distanceToHomeThreshold = distanceToHomeThreshold
        self._rescueRelAlt = rescueRelAlt
        self._homePosition = None
        self._wingPosition = None
        self._setRescueState(rescueState.DISABLE)
        self._ellipsoidMSLConverter = EllipsoidMSLConversion()

        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/mission/clear')
        rospy.wait_for_service('/mavros/mission/push')
        self._setModeProxy = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self._missionClearProxy = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        self._missionPushProxy = rospy.ServiceProxy('/mavros/mission/push', WaypointPush, persistent=True)

        self._setRescueStateService = rospy.Service("/funnywing/setRescueState", SetBool,
                                                    self._setRescueStateServiceHandler)

        # RfCommunicationHandler automatically sends data from /mavlink/from(coming from autopilot). So publish to it.
        self._rescueStatePublisher = rospy.Publisher('/mavlink/from', Mavlink, queue_size=1)
        self._rescueStatePublisherTimer = rospy.Timer(rospy.Duration(1), self._rescueStateTimerCallback)
        self._mavProtocolObj = mavutil.mavlink.MAVLink("", mavutil.mavlink.MAV_TYPE_FIXED_WING, 1)

        self._homePosSubscriber = rospy.Subscriber("/mavros/home_position/home", HomePosition, self._getHomePosition)
        self._wingPosSubscriber = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._getWingPosition)
        self._wingRelAltSubscriber = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self._getWingRelAlt)
        self._wingRcInSubscriber = rospy.Subscriber("/mavros/rc/in", RCIn, self._getWingRcIn)
        self._rescueThread = threading.Thread(target=self._rescueThreadWorker)
        self._rescueThread.start()
        return

    def _setRescueState(self, rescueState: rescueState):
        self._rescueState = rescueState
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
        try:
            if msg.channels[self._rcSafetyChannelIndex] >= 1500 and self._rcIn[self._rcSafetyChannelIndex] < 1500:
                self._setRescueState(rescueState.ENABLE)
            elif msg.channels[self._rcSafetyChannelIndex] < 1500 and self._rcIn[self._rcSafetyChannelIndex] >= 1500:
                self._setRescueState(rescueState.DISABLE)
        except Exception as e:
            rospy.logwarn("RC channels are not set till now!")
        self._rcIn = msg.channels
        rospy.loginfo(self._rcIn)
        return

    def _setRescueStateServiceHandler(self, setRescueStateRequest: SetBoolRequest):
        response = SetBoolResponse()
        try:
            self._setRescueState(rescueState.ENABLE if setRescueStateRequest.data else rescueState.DISABLE)
            response.success = True
        except rospy.ServiceException as e:
            response.success = False
            print(f"Service call failed: {e}")
        return response

    def _rescueStateTimerCallback(self, event=None):
        rescueStatusMavMsg = mavutil.mavlink.MAVLink_rescue_status_message(
            mavutil.mavlink.RESCUE_ENABLED if rescueState.ENABLE == self._rescueState else mavutil.mavlink.RESCUE_DISABLED)
        rescueStatusMavMsg.pack(self._mavProtocolObj)
        rescueStatusRosMsg = mavlink.convert_to_rosmsg(rescueStatusMavMsg)
        self._rescueStatePublisher.publish(rescueStatusRosMsg)
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
        request = SetModeRequest()
        request.custom_mode = mode
        rospy.loginfo(f"Change mode to {mode}, Success: {self._setModeProxy(request).mode_sent}")
        return

    def _clearCurrentMission(self):
        try:
            request = WaypointClearRequest()
            self._missionClearProxy(request)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def _createRescueMission(self):
        waypoints = []
        # create NAV_LOITER_UNLIM waypoint at the current GPS position(probably fake GPS).
        # For the mission to work correctly, it is necessary to create a dummy waypoint first and then the waypoint
        # going to be executed.
        dummyWaypoint = Waypoint()
        dummyWaypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        dummyWaypoint.command = CommandCode.NAV_WAYPOINT
        dummyWaypoint.is_current = False
        dummyWaypoint.autocontinue = True
        waypoints.append(dummyWaypoint)
        # Loiter waypoint at the current lat, lon.
        waypoint = Waypoint()
        waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint.command = CommandCode.NAV_LOITER_UNLIM
        waypoint.is_current = False
        waypoint.autocontinue = True
        waypoint.param3 = 120.0
        waypoint.x_lat = 0
        waypoint.y_long = 0
        waypoint.z_alt = self._rescueRelAlt
        waypoints.append(waypoint)
        return waypoints

    def _uploadWaypoints(self, waypoints):
        try:
            request = WaypointPushRequest()
            request.waypoints = waypoints
            request.start_index = 0
            print(f"Mission upload success: {self._missionPushProxy(request).success}")
            return True
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def _rescueThreadWorker(self):
        # TODO: you can also add rate for this loop to prevent from waisting cpu power.
        while not rospy.is_shutdown():
            # TODO: check this condition in field
            if self._rescueState is rescueState.DISABLE:
                rospy.loginfo("Rescue and Safety checks are disabled!")
            elif (self._homePosition is None) or (self._wingPosition is None):
                rospy.logwarn("Wing and Home position are still None!")
            elif (self._wingRelAlt < self._altThreshold) or (self._distanceToHome() > self._distanceToHomeThreshold):
                # Change mode to Auto, since loiter and circle does not get altitude and Guided would have conflict
                # with simpleTrackerNode running on GCS or RPI.
                self._clearCurrentMission()
                waypoints = self._createRescueMission()
                self._uploadWaypoints(waypoints)
                self._sendChangeModeRequest("AUTO")
            else:
                continue
        return


if __name__ == "__main__":
    rospy.init_node("safetyAndRescueNode", anonymous=True)
    safetyAndRescueObj = safetyAndRecueClass(rcSafetyChannel=7, altThreshold=15, distanceToHomeThreshold=2000,
                                             rescueRelAlt=40)
    rospy.spin()
