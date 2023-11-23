import threading

import numpy as np
import pymap3d
import rospy
from sensor_msgs.msg import NavSatFix
from pymavlink import mavutil
from mavros import mavlink
from mavros_msgs.msg import Mavlink

from wing_modules.EllipsoidMSLConversion import EllipsoidMSLConversion
from wing_modules.simpleTracker.CommandSender import CommandSenderInterface


class SimpleTracker(object):
    def __init__(self, commandSender: CommandSenderInterface, wingGPSTopic, targetGPSTopic, virtualTargetPosTopic,
                 virtualTargetOffset=120, local=False, rfConnectionTopic="/mavlink/from"):
        self._commandSender = commandSender
        self._virtualTargetOffset = virtualTargetOffset
        self._last_fw_global_pos = None
        self._virtualAroundFunnywing = True
        self._ellipsoidMSLConverter = EllipsoidMSLConversion()
        self._local = local
        if local:
            # Publishes virtual target position to RF communication to send it to GCS
            self._virtualTargetPosPublisher = rospy.Publisher(rfConnectionTopic, Mavlink, queue_size=1)
            # In local mode, we send data from MAV_TYPE_FIXED_WING, so we use this object to create Mavlink messages.
            self._mavProtocolObj = mavutil.mavlink.MAVLink("", mavutil.mavlink.MAV_TYPE_FIXED_WING, 1)
        else:
            # Publishes virtual target position in GCS
            self._virtualTargetPosPublisher = rospy.Publisher(virtualTargetPosTopic, NavSatFix, queue_size=1)
        self._wingGPSSubscriber = rospy.Subscriber(wingGPSTopic, NavSatFix, callback=self._wingGPSCallback)
        self._targetGPSSubscriber = rospy.Subscriber(targetGPSTopic, NavSatFix, callback=self._targetGPSCallback)
        self._subscriptionThread = threading.Thread(target=self._subscriptionThreadCallback)
        return

    def useWingForVirtualTargetCenter(self, useWing):
        self._virtualAroundFunnywing = useWing

    def _getVirtualTargetGlobalPosition(self, tg_global, fw_global, offset):
        """
        Gives a virtual target ahead of the funnywing to track. Heights are in MSL height. Input/Output height is in
        MSL.

        :param offset: Offset ahead of the funnywing or target to follow.
        """
        tg_global_wgs = self._ellipsoidMSLConverter.mslToEllipsoid(tg_global)
        fw_global_wgs = self._ellipsoidMSLConverter.mslToEllipsoid(fw_global)

        tg_local_pos = pymap3d.geodetic2ecef(tg_global_wgs[0], tg_global_wgs[1], tg_global_wgs[2])
        fw_local_pos = pymap3d.geodetic2ecef(fw_global_wgs[0], fw_global_wgs[1], fw_global_wgs[2])

        diff_vec = np.array(tg_local_pos) - np.array(fw_local_pos)
        diff_vec_norm = np.linalg.norm(diff_vec)
        diff_unit_vec = diff_vec / diff_vec_norm

        virt_tg_local_pos = (fw_local_pos if self._virtualAroundFunnywing else tg_local_pos) + offset * diff_unit_vec

        virt_tg_global_pos_wgs = pymap3d.ecef2geodetic(virt_tg_local_pos[0], virt_tg_local_pos[1], virt_tg_local_pos[2])

        virt_tg_global_pos_msl = self._ellipsoidMSLConverter.EllipsoidToMsl(virt_tg_global_pos_wgs)

        return virt_tg_global_pos_msl

    def _wingGPSCallback(self, msg: NavSatFix):
        self._last_fw_global_pos = [msg.latitude, msg.longitude, msg.altitude]
        return

    def _targetGPSCallback(self, msg: NavSatFix):
        fw_pos = self._last_fw_global_pos
        if fw_pos is not None:
            tg_pos = [msg.latitude, msg.longitude, msg.altitude]
            virtual_tg_pos = self._getVirtualTargetGlobalPosition(tg_pos, fw_pos, self._virtualTargetOffset)
            self._publishVirtualTargetPos(virtual_tg_pos)
            self._commandSender.sendCommand(virtual_tg_pos)
        else:
            rospy.loginfo("The funnywing gps is not subscribed yet!")
        return

    def _subscriptionThreadCallback(self):
        rospy.spin()
        return

    def _publishVirtualTargetPos(self, virtual_tg_pos):
        rosMsg = self._createRosMsg(virtual_tg_pos)
        self._virtualTargetPosPublisher.publish(rosMsg)
        return

    def _createRosMsg(self, virtual_tg_pos):
        if self._local:
            mavMsg = mavutil.mavlink.MAVLink_virtual_target_global_position_int_message(0,
                                                                                        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                                                                                        0, int(virtual_tg_pos[0] * 1e7),
                                                                                        int(virtual_tg_pos[1] * 1e7),
                                                                                        virtual_tg_pos[2], 0, 0, 0)
            mavMsg.pack(self._mavProtocolObj)
            rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        else:
            rosMsg = NavSatFix()
            rosMsg.latitude, rosMsg.longitude, rosMsg.altitude = virtual_tg_pos
        return rosMsg
