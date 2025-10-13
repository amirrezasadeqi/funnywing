import math

import rospy
from PySide2.QtCore import QObject, Slot
from PySide2.QtQml import QQmlApplicationEngine
from PySide2.QtWidgets import QApplication
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from mavros_msgs.msg import OverrideRCIn

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface
from wing_modules.CameraInterface.CameraMonitorFrameProvider import CameraMonitorFrameProvider
from wing_modules.CameraInterface.FrameProcessor import FrameProcessor
from .backFrontEndCommunication import backFrontEndCommunication
from .dataUpdater import dataUpdater

class backEnd(QObject):
    def __init__(self, qmlEngine: QQmlApplicationEngine, dataSubscriptionConfig, systemID, componentID, tgSystemID,
                 tgComponentID, gcsFromTopic="/GCS/from"):
        super().__init__()
        self._rcOverridePublisher = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self._joystick_subscriber = rospy.Subscriber(gcsFromTopic, OverrideRCIn, self._joystick_callback)

        self._qmlEngine = qmlEngine
        self._dataSubscriptionConfig = dataSubscriptionConfig
        self._systemID = systemID
        self._componentID = componentID
        self._tgSystemID = tgSystemID
        self._tgComponentID = tgComponentID
        self._cameraMonitorFrameProvider = None
        # Initialize with -1, so if there has been no tracks, the sensor block does not lock, since track_id -1 can't be
        # existed.
        self._lastTrackId = -1

        # Setup signals and connections
        self._backFrontConnection = backFrontEndCommunication()
        self._qmlEngine.rootContext().setContextProperty("backFrontConnections", self._backFrontConnection)
        self._backFrontConnection.setArmStateSignal.connect(self.pubArmDisarmCommand)
        self._backFrontConnection.setFlightModeSignal.connect(self.pubSetModeCommand)
        self._backFrontConnection.goToLocationSignal.connect(self.pubGoToCommand)
        self._backFrontConnection.sendSetRescueStatusSignal.connect(self.pubSetRescueStateCommand)
        self._backFrontConnection.handleTestScenarioSignal.connect(self.handleTestScenario)
        self._backFrontConnection.setSimpleTrackerSettingsSignal.connect(self.setSimpleTrackerSettings)
        self._backFrontConnection.setSimpleTrackerActivationSignal.connect(self.setSimpleTrackerActivation)
        self._backFrontConnection.setArduplaneParamSignal.connect(self.setArduplaneParameter)
        self._backFrontConnection.closeBackendSignal.connect(self.closeBackend)
        self._backFrontConnection.setZoomPercentageSignal.connect(self.setZoomPercentage)
        self._backFrontConnection.trackLockSignal.connect(self.sendLockOnTrackCommand)
        self._backFrontConnection.setVisualTrackerSettingsSignal.connect(self.setVisualTrackerSettings)
        self._backFrontConnection.setCameraBasedGuiderConfigsSignal.connect(self.setCameraBasedGuiderConfigs)
        self._backFrontConnection.setLastTrackIdSignal.connect(self.setLastTrackId)

        self._dataUpdater = dataUpdater(self._dataSubscriptionConfig, self._backFrontConnection)
        # TODO[test needed]: MAVLink object does not try to connect to the connection string and
        #   I don't know if the connection string is important in de/serialization. So I will use
        #   empty string now and if it will be ok delete this, otherwise we must pass the address
        #   of the connection(since MAVLink does not connect to that automatically, I think there
        #   will be no problem about occupied connection).
        
        self._protocolObj = mavutil.mavlink.MAVLink('', self._systemID, self._componentID)
        # Publisher for sending mavlink Commands and all the data which is needed in the RPI side.
        self._toRfComPublisher = rospy.Publisher(gcsFromTopic, Mavlink, queue_size=10)
        
        return

    ARDUPLANE_MODE_MAP = {
        "MANUAL": 0,
        "CIRCLE": 1,
        "STABILIZE": 2,
        "TRAINING": 3,
        "ACRO": 4,
        "FBWA": 5,
        "FBWB": 6,
        "CRUISE": 7,
        "AUTOTUNE": 8,
        "AUTO": 10,
        "RTL": 11,
        "LOITER": 12,
        "GUIDED": 15
    }
    
    def _joystick_callback(self, msg: OverrideRCIn):
        self._rcOverridePublisher.publish(msg)
        return

    def createAndSetupFrameProvider(self, frameCapture: CameraFrameCaptureInterface, qtApplication: QApplication):
        self._cameraMonitorFrameProvider = CameraMonitorFrameProvider(frame_capture=frameCapture,
                                                                      backFrontConnection=self._backFrontConnection)
        self._qmlEngine.addImageProvider("cameraMonitorFrameProvider", self._cameraMonitorFrameProvider)
        # close.accepted = false on QML front-end, holds the front-end up till the back-end to be closed. so,
        # we need to close the app using its quit slot, after the camera monitor frame provider was stopped.
        # Note: we may move this line in future to the FieldTestAppNode for better logic!!!
        self._cameraMonitorFrameProvider.cameraMonitorFrameProviderQuited.connect(qtApplication.quit)
        return

    def setupConnectionWithFrameProcessor(self, frame_processor: FrameProcessor):
        frame_processor.setQtCommunicator(self._backFrontConnection)
        return

    @Slot(bool)
    def pubArmDisarmCommand(self, armState):
        # create mavlink message
        mavMsg = mavutil.mavlink.MAVLink_command_long_message(self._tgSystemID, self._tgComponentID,
                                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                                              1 if armState else 0, 0, 0, 0, 0, 0, 0)
        # convert it to mavros_msgs/Mavlink message
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        # publish the message, and it will automatically be sent.
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(str)
    def pubSetModeCommand(self, flightMode):
        # Note that command's param1 should be set, otherwise Arduplane does not change the mode.
        mavMsg = mavutil.mavlink.MAVLink_command_long_message(self._tgSystemID, self._tgComponentID,
                                                              mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                              self.ARDUPLANE_MODE_MAP[flightMode], 0, 0, 0, 0, 0)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(float, float, float)
    def pubGoToCommand(self, lat, lon, alt):
        # Scaling lat, lon to use them with MAVLink_command_int_message.
        lat = int(lat * 1e7)
        lon = int(lon * 1e7)
        mavMsg = mavutil.mavlink.MAVLink_command_int_message(self._tgSystemID, self._tgComponentID,
                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                             mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1,
                                                             mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 120,
                                                             0, lat, lon, alt)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(bool)
    def pubSetRescueStateCommand(self, rescueState):
        mavMsg = mavutil.mavlink.MAVLink_rescue_set_state_message(
            mavutil.mavlink.RESCUE_ENABLED if rescueState else mavutil.mavlink.RESCUE_DISABLED)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(int, bool)
    def handleTestScenario(self, scenarioIdx, active):
        int_params = [0] * 5
        bool_params = [False] * 5
        float_params = [0.0] * 5
        int_params[0] = scenarioIdx
        bool_params[0] = active
        mavMsg = mavutil.mavlink.MAVLink_funnywing_custom_command_message(self._tgSystemID, self._tgComponentID,
                                                                          mavutil.mavlink.SET_TEST_SCENARIO_ACTIVATION,
                                                                          int_params, bool_params, float_params)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(float, bool, bool)
    def setSimpleTrackerSettings(self, waypointRadius, local, wingAsVirtualCenter):
        int_params = [0] * 5
        bool_params = [False] * 5
        float_params = [0.0] * 5
        float_params[0] = waypointRadius
        bool_params[0] = local
        bool_params[1] = wingAsVirtualCenter
        mavMsg = mavutil.mavlink.MAVLink_funnywing_custom_command_message(self._tgSystemID, self._tgComponentID,
                                                                          mavutil.mavlink.SET_SIMPLE_TRACKER_SETTINGS,
                                                                          int_params, bool_params, float_params)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(bool)
    def setSimpleTrackerActivation(self, active):
        int_params = [0] * 5
        bool_params = [False] * 5
        float_params = [0.0] * 5
        bool_params[0] = active
        mavMsg = mavutil.mavlink.MAVLink_funnywing_custom_command_message(self._tgSystemID, self._tgComponentID,
                                                                          mavutil.mavlink.SET_SIMPLE_TRACKER_ACTIVATION,
                                                                          int_params, bool_params, float_params)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(str, float)
    def setArduplaneParameter(self, paramName, paramValue):
        # create mavlink message
        mavMsg = mavutil.mavlink.MAVLink_param_set_message(self._tgSystemID, self._tgComponentID, paramName.encode(),
                                                           paramValue, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # convert it to mavros_msgs/Mavlink message
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        # publish the message, and it will automatically be sent.
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot()
    def closeBackend(self):
        self._dataUpdater.stop()
        return

    @Slot(int)
    def setZoomPercentage(self, zoom_percentage):
        int_params = [0] * 5
        bool_params = [False] * 5
        float_params = [0.0] * 5
        int_params[0] = zoom_percentage
        mavMsg = mavutil.mavlink.MAVLink_funnywing_custom_command_message(self._tgSystemID, self._tgComponentID,
                                                                          mavutil.mavlink.SET_CAMERA_PRESET_INDEX,
                                                                          int_params, bool_params, float_params)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(bool, int)
    def sendLockOnTrackCommand(self, locked, track_id):
        int_params = [0] * 5
        bool_params = [False] * 5
        float_params = [0.0] * 5
        bool_params[0] = locked
        # Locking on the last track if the signal contains -1 as the track_id argument, otherwise locking
        # on the commanded track_id.
        int_params[0] = track_id if track_id != -1 else self._lastTrackId
        mavMsg = mavutil.mavlink.MAVLink_funnywing_custom_command_message(self._tgSystemID, self._tgComponentID,
                                                                          mavutil.mavlink.LOCK_ON_TRACK,
                                                                          int_params, bool_params, float_params)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(float, int, int)
    def setVisualTrackerSettings(self, distThresh, initDelay, hitCountMax):
        int_params = [0] * 5
        bool_params = [False] * 5
        float_params = [0.0] * 5
        int_params[0] = initDelay
        int_params[1] = hitCountMax
        float_params[0] = distThresh
        mavMsg = mavutil.mavlink.MAVLink_funnywing_custom_command_message(self._tgSystemID, self._tgComponentID,
                                                                          mavutil.mavlink.SET_TRACKER_CONFIGS,
                                                                          int_params, bool_params, float_params)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot('QVariantMap')
    def setCameraBasedGuiderConfigs(self, configs):
        profile_type = mavutil.mavlink.CUSTOM_SIGMOID if int(configs.get("profile_type")) else mavutil.mavlink.CONSTANT
        const_throttle = float(configs.get("const_throttle"))
        const_throttle = const_throttle if (not math.isnan(const_throttle)) and (0 <= const_throttle <= 1) else 0.5
        x_pids = configs.get("x_pids")
        xpid_saturations = configs.get("xpid_saturations")
        y_pids = configs.get("y_pids")
        ypid_saturations = configs.get("ypid_saturations")
        a = configs.get("a")
        b = configs.get("b")
        wing_too_below_throttle = configs.get("wing_too_below_throttle")
        wing_too_above_throttle = configs.get("wing_too_above_throttle")
        wing_tg_at_same_level_throttle = configs.get("wing_tg_at_same_level_throttle")
        size_threshold = int(configs.get("size_threshold"))
        wing_too_below_threshold = int(configs.get("wing_too_below_threshold"))
        wing_too_above_threshold = int(configs.get("wing_too_above_threshold"))
        mavMsgFields = [
            profile_type,
            const_throttle,
            x_pids,
            xpid_saturations,
            y_pids,
            ypid_saturations,
            a,
            b,
            wing_too_below_throttle,
            wing_too_above_throttle,
            wing_tg_at_same_level_throttle,
            size_threshold,
            wing_too_below_threshold,
            wing_too_above_threshold
        ]
        mavMsg = mavutil.mavlink.MAVLink_set_cam_based_guidance_configs_message(*mavMsgFields)
        mavMsg.pack(self._protocolObj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        self._toRfComPublisher.publish(rosMsg)
        return

    @Slot(int)
    def setLastTrackId(self, lastTrackId):
        self._lastTrackId = lastTrackId
        return
