import rospy
from pymavlink import mavutil
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from wing_navigator.srv import SetSimpleTrackerSettings, SetSimpleTrackerSettingsRequest, \
    SetSimpleTrackerSettingsResponse, RunTestScenario, RunTestScenarioRequest, RunTestScenarioResponse

from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


# TODO: Redesign and Refactor this File.

def setSimpleTrackerSettingsHandler(mavMsg):
    request = SetSimpleTrackerSettingsRequest()
    request.waypointRadius = mavMsg.float_params[0]
    request.wingAsVirtualCenter = mavMsg.bool_params[1]
    response = SetSimpleTrackerSettingsResponse()
    proxy = funnywing_custom_command_job._setSimpleTrackerSettingsProxy
    return request, response, proxy


def setSimpleTrackerActivationHandler(mavMsg):
    request = SetBoolRequest()
    request.data = mavMsg.bool_params[0]
    response = SetBoolResponse()
    proxy = funnywing_custom_command_job._activeSimpleTrackerProxy
    return request, response, proxy


def setTestScenarioActivation(mavMsg):
    request = RunTestScenarioRequest()
    request.scenarioIdx = mavMsg.int_params[0]
    request.active = mavMsg.bool_params[0]
    response = RunTestScenarioResponse()
    proxy = funnywing_custom_command_job._runTestScenarioProxy
    return request, response, proxy


funnywingCustomCommandHandlerMapping = {
    mavutil.mavlink.SET_SIMPLE_TRACKER_SETTINGS: setSimpleTrackerSettingsHandler,
    mavutil.mavlink.SET_SIMPLE_TRACKER_ACTIVATION: setSimpleTrackerActivationHandler,
    mavutil.mavlink.SET_TEST_SCENARIO_ACTIVATION: setTestScenarioActivation
}


class funnywing_custom_command_job(JobInterface):
    # TODO: Rethink about the design for creating these proxies and check that they works correctly. Specially the way
    #   we have created them in the constructor.
    _activeSimpleTrackerProxy = rospy.ServiceProxy("/funnywing/activeSimpleTracker", SetBool)
    _setSimpleTrackerSettingsProxy = rospy.ServiceProxy("/funnywing/setSimpleTrackerSettings", SetSimpleTrackerSettings)
    _runTestScenarioProxy = rospy.ServiceProxy("/funnywing/runTestScenario", RunTestScenario)

    def __init__(self, message, rfConnection: ConnectionInterface, system, component):
        """

        @type message: MAVLink_funnywing_custom_command_message
        """
        super().__init__(message, rfConnection, system, component)
        if funnywing_custom_command_job._activeSimpleTrackerProxy is None:
            rospy.wait_for_service("/funnywing/activeSimpleTracker")
            funnywing_custom_command_job._activeSimpleTrackerProxy = rospy.ServiceProxy(
                "/funnywing/activeSimpleTracker", SetBool)
        elif funnywing_custom_command_job._setSimpleTrackerSettingsProxy is None:
            rospy.wait_for_service("/funnywing/setSimpleTrackerSettings")
            funnywing_custom_command_job._setSimpleTrackerSettingsProxy = rospy.ServiceProxy(
                "/funnywing/setSimpleTrackerSettings", SetSimpleTrackerSettings)
        elif funnywing_custom_command_job._runTestScenarioProxy is None:
            rospy.wait_for_service("/funnywing/runTestScenario")
            funnywing_custom_command_job._runTestScenarioProxy = rospy.ServiceProxy(
                "/funnywing/runTestScenario", RunTestScenario)

        self._handler = funnywingCustomCommandHandlerMapping[self.getMessage().command]
        self._request, self._response, self._proxy = self._handler(self.getMessage())
        return

    def _doJob(self):
        self._response = self._proxy(self._request)
        rospy.loginfo(f"{self._response}")
        return
