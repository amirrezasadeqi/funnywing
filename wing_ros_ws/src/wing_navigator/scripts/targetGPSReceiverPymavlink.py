#!/usr/bin/env python

"""
This file is used for getting target GPS by creating a mavlink connection via Pymavlink library,
so we can use all types of physical connection, e.g. serial, UDP, TCP and so on. Then this data
if feed into GUI and RF connection.
"""

import argparse
import threading

import rospy
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix


class GPSDataReceiverPymavlink(object):
    def __init__(self, connection_string, gpsTopicName, rfConnectionTopicName, gpsDataRate):
        """

        @param connection_string: This string format is documented in mavlink docs. But for convenience, for serial
        connection, we use this different format: "<serial_device>:<baudrate>", e.g. "/dev/ttyUSB0:57600".
        """
        self._connection = None
        self._gpsDataRate = gpsDataRate
        self._construct_mavlink_connection(connection_string)
        self._set_gps_mavlink_msg_rate()
        # Creating MAVLink object to construct Mavlink message from mavlink one. Note that the code is running on GCS
        # so the sender or source system is GCS.
        self._mavProtocolObj = mavutil.mavlink.MAVLink("", mavutil.mavlink.MAV_TYPE_GCS, 1)

        self._receiveLoopRate = rospy.Rate(self._gpsDataRate)

        self._GPSPublisher = rospy.Publisher(gpsTopicName, NavSatFix, queue_size=1)
        self._GPSToRfConnectionPublisher = rospy.Publisher(rfConnectionTopicName, Mavlink, queue_size=1)
        self._GPSPublishBuffer = []
        self._publishLoopRate = rospy.Rate(self._gpsDataRate)
        self._GPSPublisherThread = threading.Thread(target=self._GPSPublisherThreadWorker)
        self._GPSPublisherThread.start()
        return

    def GPSDataReceiveLoop(self):
        while not rospy.is_shutdown():
            try:
                msg = self._connection.recv_match(type="GPS_RAW_INT", blocking=True)
                if msg:
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1e3
                    self._GPSPublishBuffer.append([lat, lon, alt])
            except KeyboardInterrupt:
                rospy.logwarn("Exiting GPS receiving loop by keyboard interrupt ...")
                break
            except Exception as e:
                rospy.logwarn(f"Error: {e}")
            self._receiveLoopRate.sleep()
        return

    def _construct_mavlink_connection(self, connection_string: str):
        if connection_string.startswith("/dev/tty"):  # serial connection, e.g. 9XTend
            dev = connection_string.split(":")[0]
            baud = connection_string.split(":")[1]
            self._connection = mavutil.mavlink_connection(dev, baud=baud)
        else:  # UDP, TCP and etc. connection.
            self._connection = mavutil.mavlink_connection(connection_string)

        self._connection.wait_heartbeat()
        print(
            f"Heartbeat from system: {self._connection.target_system}, component: {self._connection.target_component}")
        return

    def _set_gps_mavlink_msg_rate(self):
        """
        This sends a set message interval mavlink command to the target pymavlink connection, for setting the
        rate of the GPS message.
        """
        msg = self._connection.mav.command_long_encode(
            self._connection.target_system,
            self._connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            1000000 / self._gpsDataRate,
            0, 0, 0, 0, 0
        )
        self._connection.mav.send(msg)
        return

    def _GPSPublisherThreadWorker(self):
        while not rospy.is_shutdown():
            if len(self._GPSPublishBuffer):
                tgGPSPosition = self._GPSPublishBuffer.pop(0)
                self._publishToGCSRosNetwork(tgGPSPosition)
                self._publishToRfConnection(tgGPSPosition)
            else:
                rospy.loginfo("Target GPS publish buffer is empty!")
            self._publishLoopRate.sleep()
        return

    def _publishToGCSRosNetwork(self, tgGPSPosition):
        # TODO: Construct the header in future if it is needed.
        msg = NavSatFix()
        msg.latitude, msg.longitude, msg.altitude = tgGPSPosition
        self._GPSPublisher.publish(msg)
        return

    def _publishToRfConnection(self, tgGPSPosition):
        msg = self._constructMavlinkMsg(tgGPSPosition)
        self._GPSToRfConnectionPublisher.publish(msg)
        return

    def _constructMavlinkMsg(self, tgGPSPosition):
        # For now, we only set position fields and other fields are set to zero.
        # TODO: In future if you need, you should set first field that is timestamp and also other data fields like
        #   velocity fields if they are needed.
        mavMsg = mavutil.mavlink.MAVLink_position_target_global_int_message(0, mavutil.mavlink.MAV_FRAME_GLOBAL_INT, 0,
                                                                            int(tgGPSPosition[0] * 1e7),
                                                                            int(tgGPSPosition[1] * 1e7),
                                                                            tgGPSPosition[2], 0, 0, 0, 0, 0, 0, 0, 0)
        mavMsg.pack(self._mavProtocolObj)
        return mavlink.convert_to_rosmsg(mavMsg)


if __name__ == "__main__":
    rospy.init_node("targetGPSReceiverPymavlink", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connection_string", default="udpin:0.0.0.0:14551")
    parser.add_argument("-t", "--gpsTopicName", default="/target/globalPosition")
    parser.add_argument("-r", "--rfConnectionTopicName", default="/GCS/from")
    parser.add_argument("-f", "--gpsDataFreq", default=2)
    args, unknown = parser.parse_known_args()

    gpsDataReceiver = GPSDataReceiverPymavlink(args.connection_string, args.gpsTopicName, args.rfConnectionTopicName,
                                               args.gpsDataFreq)
    # Note: Receiver loop blocks the main thread execution.
    gpsDataReceiver.GPSDataReceiveLoop()
