#!/usr/bin/env python

import threading

import rospy
from mavros_msgs.msg import Waypoint, CommandCode, WaypointReached
from mavros_msgs.srv import SetMode, WaypointClear, WaypointPush, SetModeRequest, CommandInt, CommandIntRequest, \
    WaypointClearRequest, WaypointPushRequest
from pymavlink import mavutil
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from wing_navigator.srv import SetSimpleTrackerSettings, SetSimpleTrackerSettingsRequest, \
    SetSimpleTrackerSettingsResponse, RunTestScenario, RunTestScenarioRequest, RunTestScenarioResponse

from wing_modules.simpleTracker.CommandSender import CommandSenderLocal
from wing_modules.simpleTracker.SimpleTracker import SimpleTracker


class NavigatorNode(object):
    def __init__(self):
        self._currentWaypoint = 0
        self._reachedWaypointSubscriber = rospy.Subscriber("/mavros/mission/reached", WaypointReached,
                                                           callback=self._waypointReachedCallback)
        self._constructSimpleTrackerObj()
        self._constructTestScenarioMapping()

        self._activeSimpleTrackerService = rospy.Service("/funnywing/activeSimpleTracker", SetBool,
                                                         self._activeSimpleTrackerServiceHandler)
        self._setSimpleTrackerSettingsService = rospy.Service("/funnywing/setSimpleTrackerSettings",
                                                              SetSimpleTrackerSettings,
                                                              self._setSimpleTrackerSettingsServiceHandler)
        self._runTestScenarioService = rospy.Service("/funnywing/runTestScenario", RunTestScenario,
                                                     self._runTestScenarioServiceHandler)

        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/mission/clear')
        rospy.wait_for_service('/mavros/mission/push')
        rospy.wait_for_service("/mavros/cmd/command_int")
        self._setModeProxy = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self._missionClearProxy = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        self._missionPushProxy = rospy.ServiceProxy('/mavros/mission/push', WaypointPush, persistent=True)
        self._comIntProxy = rospy.ServiceProxy("/mavros/cmd/command_int", CommandInt)

        return

    def _setCurrentWaypoint(self, sequence):
        self._currentWaypoint = sequence
        return

    def _waypointReachedCallback(self, msg: WaypointReached):
        self._setCurrentWaypoint(msg.wp_seq)
        return

    def _constructSimpleTrackerObj(self):
        waypointRadius = 120
        radiusOffset = 5
        self._simpleTrackerCommandSender = CommandSenderLocal()
        self._simpleTrackerCommandSender.setWayPointRadius(waypointRadius)
        self._simpleTracker = SimpleTracker(self._simpleTrackerCommandSender, "/mavros/global_position/global",
                                            "/funnywing/targetGlobalPosition", "/virtualTarget/globalPosition",
                                            virtualTargetOffset=waypointRadius + radiusOffset, local=True,
                                            rfConnectionTopic="/mavlink/from")
        self._simpleTracker.useWingForVirtualTargetCenter(True)
        self._simpleTracker.setActivated(False)
        return

    def _constructTestScenarioMapping(self):
        self._testScenarioMapping = {
            1: {
                "active": self._testScenario1ActiveHandler,
                "inactive": self._testScenario1InactiveHandler
            }
        }
        return

    def _activeSimpleTrackerServiceHandler(self, request: SetBoolRequest):
        response = SetBoolResponse()
        try:
            self._simpleTracker.setActivated(request.data)
            response.success = True
        except rospy.ServiceException as e:
            response.success = False
            print(f"Service call failed: {e}")
        return response

    def _setSimpleTrackerSettingsServiceHandler(self, request: SetSimpleTrackerSettingsRequest):
        response = SetSimpleTrackerSettingsResponse()
        try:
            self._simpleTracker.setSettings(request.wingAsVirtualCenter, request.waypointRadius)
            response.success = True
        except rospy.ServiceException as e:
            response.success = False
            print(f"Service call failed: {e}")
        return response

    def _runTestScenarioServiceHandler(self, request: RunTestScenarioRequest):
        response = RunTestScenarioResponse()
        try:
            handler = self._testScenarioMapping[request.scenarioIdx]["active" if request.active else "inactive"]
            handlerThread = threading.Thread(target=handler)
            handlerThread.start()
            response.success = True
        except rospy.ServiceException as e:
            response.success = False
            print(f"Service call failed: {e}")
        return response

    def _testScenario1ActiveHandler(self):
        request = WaypointClearRequest()
        self._missionClearProxy(request)
        waypoints = self._createTestScenarioMission()
        self._uploadWaypoints(waypoints)
        # To ignore the last waypoint reached message from the previous mission.
        self._setCurrentWaypoint(0)
        request = SetModeRequest()
        request.custom_mode = "AUTO"
        self._setModeProxy(request)

        # Hold until the wing reaches last waypoint
        while 3 != self._currentWaypoint:
            rospy.sleep(0.1)

        self._simpleTracker.setActivated(True)

        request = SetModeRequest()
        request.custom_mode = "GUIDED"
        self._setModeProxy(request)
        return

    def _testScenario1InactiveHandler(self):
        self._simpleTracker.setActivated(False)

        # Change to guided mode, since in the test scenario(AUTO mode) the following command rejected by the autopilot
        # (I think) and the scenario mission does not stop and not being cleared.
        request = SetModeRequest()
        request.custom_mode = "GUIDED"
        self._setModeProxy(request)

        request = CommandIntRequest()
        request.broadcast = 0
        request.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        request.command = mavutil.mavlink.MAV_CMD_DO_REPOSITION
        request.current = False
        request.autocontinue = False
        request.param1 = -1
        request.param2 = mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
        request.param3 = 80.0
        request.param4 = 0
        request.x = 0
        request.y = 0
        request.z = 40
        self._comIntProxy(request)

        request = WaypointClearRequest()
        self._missionClearProxy(request)
        return

    def _createTestScenarioMission(self):
        waypoints = []
        mission_rel_alt = 40
        waypointRadius = self._simpleTrackerCommandSender._wayPointRadius
        dummyWaypoint = Waypoint()
        dummyWaypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        dummyWaypoint.command = CommandCode.NAV_WAYPOINT
        dummyWaypoint.is_current = False
        dummyWaypoint.autocontinue = True
        waypoints.append(dummyWaypoint)
        # Waypoint 1
        waypoint = Waypoint()
        waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint.command = CommandCode.NAV_WAYPOINT
        waypoint.is_current = False
        waypoint.autocontinue = True
        waypoint.param3 = waypointRadius
        waypoint.x_lat = 35.74805410878459
        waypoint.y_long = 51.60696543545722
        waypoint.z_alt = mission_rel_alt
        waypoints.append(waypoint)
        # Waypoint 2
        waypoint = Waypoint()
        waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint.command = CommandCode.NAV_WAYPOINT
        waypoint.is_current = False
        waypoint.autocontinue = True
        waypoint.param3 = waypointRadius
        waypoint.x_lat = 35.748078036064534
        waypoint.y_long = 51.60518915432342
        waypoint.z_alt = mission_rel_alt
        waypoints.append(waypoint)
        # Waypoint 3
        waypoint = Waypoint()
        waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint.command = CommandCode.NAV_WAYPOINT
        waypoint.is_current = False
        waypoint.autocontinue = True
        waypoint.param3 = waypointRadius
        waypoint.x_lat = 35.74807205424523
        waypoint.y_long = 51.60364872794597
        waypoint.z_alt = mission_rel_alt
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


if __name__ == '__main__':
    rospy.init_node('navigatorNode', anonymous=True)
    navigatorNode = NavigatorNode()
    rospy.spin()
