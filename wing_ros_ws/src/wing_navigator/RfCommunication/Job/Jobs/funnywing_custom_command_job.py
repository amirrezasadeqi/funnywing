import rospy
from pymavlink import mavutil
from std_srvs.srv import SetBool, SetBoolRequest
from wing_navigator.srv import SetDouble, SetDoubleRequest
from wing_navigator.srv import SetSimpleTrackerSettings, SetSimpleTrackerSettingsRequest, \
    RunTestScenario, RunTestScenarioRequest, LockOnOff, LockOnOffRequest, SetVisualTrackerConfigs, \
    SetVisualTrackerConfigsRequest

from RfCommunication.Job.Interface.JobInterface import JobInterface
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


# TODO: Redesign and Refactor this File.

def setSimpleTrackerSettingsHandler(mavMsg):
    request = SetSimpleTrackerSettingsRequest()
    request.waypointRadius = mavMsg.float_params[0]
    request.wingAsVirtualCenter = mavMsg.bool_params[1]
    proxy = funnywing_custom_command_job.setSimpleTrackerSettingsProxy
    response = proxy(request)
    rospy.loginfo(f"{response}")
    return


def setSimpleTrackerActivationHandler(mavMsg):
    request = SetBoolRequest()
    request.data = mavMsg.bool_params[0]
    proxy = funnywing_custom_command_job.activeSimpleTrackerProxy
    response = proxy(request)
    rospy.loginfo(f"{response}")
    return


def setTestScenarioActivation(mavMsg):
    request = RunTestScenarioRequest()
    request.scenarioIdx = mavMsg.int_params[0]
    request.active = mavMsg.bool_params[0]
    proxy = funnywing_custom_command_job.runTestScenarioProxy
    response = proxy(request)
    rospy.loginfo(f"{response}")
    return


def setCameraPresetIndexHandler(mavMsg):
    # Using the SetDouble for setting integer value to not have to create another service for setting integer values.
    request = SetDoubleRequest()
    # [0-100] preset index converted to float to be able to use the SetDouble service.
    request.data = float(mavMsg.int_params[0])
    proxy = funnywing_custom_command_job.setCameraPresetIndexProxy
    response = proxy(request)
    rospy.loginfo(f"{response}")
    return


def lockOnTrackHandler(mavMsg):
    request = LockOnOffRequest()
    request.lock_on = bool(mavMsg.bool_params[0])
    request.track_id = int(mavMsg.int_params[0])
    proxy = funnywing_custom_command_job.lockOnOffProxy
    response = proxy(request)
    rospy.loginfo(f"{response}")
    return


def setTrackerConfigsHandler(mavMsg):
    request = SetVisualTrackerConfigsRequest()
    request.dist_thresh = float(mavMsg.float_params[0])
    request.init_delay = int(mavMsg.int_params[0])
    request.hit_count_max = int(mavMsg.int_params[1])
    proxy = funnywing_custom_command_job.setTrackerConfigsProxy
    response = proxy(request)
    rospy.loginfo(f"{response}")
    return


funnywingCustomCommandHandlerMapping = {
    mavutil.mavlink.SET_SIMPLE_TRACKER_SETTINGS: setSimpleTrackerSettingsHandler,
    mavutil.mavlink.SET_SIMPLE_TRACKER_ACTIVATION: setSimpleTrackerActivationHandler,
    mavutil.mavlink.SET_TEST_SCENARIO_ACTIVATION: setTestScenarioActivation,
    mavutil.mavlink.SET_CAMERA_PRESET_INDEX: setCameraPresetIndexHandler,
    mavutil.mavlink.LOCK_ON_TRACK: lockOnTrackHandler,
    mavutil.mavlink.SET_TRACKER_CONFIGS: setTrackerConfigsHandler
}


class funnywing_custom_command_job(JobInterface):
    # TODO: Rethink about the design for creating these proxies and check that they works correctly. Specially the way
    #   we have created them in the constructor.
    activeSimpleTrackerProxy = rospy.ServiceProxy("/funnywing/activeSimpleTracker", SetBool)
    setSimpleTrackerSettingsProxy = rospy.ServiceProxy("/funnywing/setSimpleTrackerSettings", SetSimpleTrackerSettings)
    runTestScenarioProxy = rospy.ServiceProxy("/funnywing/runTestScenario", RunTestScenario)
    setCameraPresetIndexProxy = rospy.ServiceProxy("/funnywing/camera/set_preset_index", SetDouble)
    lockOnOffProxy = rospy.ServiceProxy("/funnywing/lock_on_off", LockOnOff)
    setTrackerConfigsProxy = rospy.ServiceProxy("/funnywing/setTrackerConfigs", SetVisualTrackerConfigs)

    def __init__(self, message, rfConnection: ConnectionInterface, system, component):
        """

        @type message: MAVLink_funnywing_custom_command_message
        """
        super().__init__(message, rfConnection, system, component)
        if funnywing_custom_command_job.activeSimpleTrackerProxy is None:
            rospy.wait_for_service("/funnywing/activeSimpleTracker")
            funnywing_custom_command_job.activeSimpleTrackerProxy = rospy.ServiceProxy(
                "/funnywing/activeSimpleTracker", SetBool)
        elif funnywing_custom_command_job.setSimpleTrackerSettingsProxy is None:
            rospy.wait_for_service("/funnywing/setSimpleTrackerSettings")
            funnywing_custom_command_job.setSimpleTrackerSettingsProxy = rospy.ServiceProxy(
                "/funnywing/setSimpleTrackerSettings", SetSimpleTrackerSettings)
        elif funnywing_custom_command_job.runTestScenarioProxy is None:
            rospy.wait_for_service("/funnywing/runTestScenario")
            funnywing_custom_command_job.runTestScenarioProxy = rospy.ServiceProxy(
                "/funnywing/runTestScenario", RunTestScenario)
        elif funnywing_custom_command_job.setCameraPresetIndexProxy is None:
            rospy.wait_for_service("/funnywing/camera/set_preset_index")
            funnywing_custom_command_job.setCameraPresetIndexProxy = rospy.ServiceProxy(
                "/funnywing/camera/set_preset_index", SetDouble)
        elif funnywing_custom_command_job.lockOnOffProxy is None:
            rospy.wait_for_service("/funnywing/lock_on_off")
            funnywing_custom_command_job.lockOnOffProxy = rospy.ServiceProxy(
                "/funnywing/lock_on_off", LockOnOff)
        elif funnywing_custom_command_job.setTrackerConfigsProxy is None:
            rospy.wait_for_service("/funnywing/setTrackerConfigs")
            funnywing_custom_command_job.setTrackerConfigsProxy = rospy.ServiceProxy(
                "/funnywing/setTrackerConfigs", SetVisualTrackerConfigs)

        self._handler = funnywingCustomCommandHandlerMapping[self.getMessage().command]
        return

    def _doJob(self):
        try:
            self._handler(self.getMessage())
        except Exception as e:
            rospy.logwarn(f"Exception occurred by funnywing_custom_command handler: {e}")
        return
