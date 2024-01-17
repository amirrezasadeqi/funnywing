import rospy
from PySide2.QtCore import QObject, Slot
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil

from .dataUpdater import dataUpdater


class backEnd(QObject):
    def __init__(self, dataSubscriptionConfig, backFrontConnection, systemID, componentID, tgSystemID, tgComponentID,
                 gcsFromTopic="/GCS/from"):
        super().__init__()
        self._dataSubscriptionConfig = dataSubscriptionConfig
        self._systemID = systemID
        self._componentID = componentID
        self._tgSystemID = tgSystemID
        self._tgComponentID = tgComponentID

        # Setup signals and connections
        self._backFrontConnection = backFrontConnection
        self._backFrontConnection.setArmStateSignal.connect(self.pubArmDisarmCommand)
        self._backFrontConnection.setFlightModeSignal.connect(self.pubSetModeCommand)
        self._backFrontConnection.goToLocationSignal.connect(self.pubGoToCommand)
        self._backFrontConnection.sendSetRescueStatusSignal.connect(self.pubSetRescueStateCommand)

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
