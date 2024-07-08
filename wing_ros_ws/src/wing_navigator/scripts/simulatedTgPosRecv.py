#!/usr/bin/env python

import argparse
import socket

import rospy
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix

from wing_modules.Tcp_message import Tcp_message


class PosClient(object):
    def __init__(self, server_ip, server_port, gpsTopicName, rfConnectionTopicName):
        self._server_ip = server_ip
        self._server_port = server_port
        # Creating MAVLink object to construct ROS Mavlink message from mavlink one. Note that the code is running on
        # GCS so the sender or source system is GCS.
        self._mavProtocolObj = mavutil.mavlink.MAVLink("", mavutil.mavlink.MAV_TYPE_GCS, 1)
        self._GPSPublisher = rospy.Publisher(gpsTopicName, NavSatFix, queue_size=1)
        self._GPSToRfConnectionPublisher = rospy.Publisher(rfConnectionTopicName, Mavlink, queue_size=1)
        self._client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        return

    def run(self):
        self._connect_to_server()
        try:
            while not rospy.is_shutdown():
                # socket.recv is blocking so there is no performance waste.
                message = Tcp_message()
                message.read_from(self._client_socket)
                tgGPSPosition = message.decode()
                self._publishToGCSRosNetwork(tgGPSPosition)
                self._publishToRfConnection(tgGPSPosition)
        except KeyboardInterrupt:
            rospy.loginfo("Client Stopping!")
        except Exception as e:
            rospy.loginfo(f"Exception occurred in reading from socket and publishing to ROS.")
        finally:
            self._close_connection()
        return

    def _connect_to_server(self):
        self._client_socket.connect((self._server_ip, self._server_port))
        rospy.loginfo(f"Connected to server at {self._server_ip}:{self._server_port}")
        return

    def _close_connection(self):
        self._client_socket.close()
        return

    def _publishToGCSRosNetwork(self, tgGPSPosition):
        msg = NavSatFix()
        msg.latitude, msg.longitude, msg.altitude = tgGPSPosition
        self._GPSPublisher.publish(msg)
        return

    def _publishToRfConnection(self, tgGPSPosition):
        msg = self._constructMavlinkMsg(tgGPSPosition)
        self._GPSToRfConnectionPublisher.publish(msg)
        return

    def _constructMavlinkMsg(self, tgGPSPosition):
        mavMsg = mavutil.mavlink.MAVLink_position_target_global_int_message(0, mavutil.mavlink.MAV_FRAME_GLOBAL_INT, 0,
                                                                            int(tgGPSPosition[0] * 1e7),
                                                                            int(tgGPSPosition[1] * 1e7),
                                                                            tgGPSPosition[2], 0, 0, 0, 0, 0, 0, 0, 0)
        mavMsg.pack(self._mavProtocolObj)
        return mavlink.convert_to_rosmsg(mavMsg)


if "__main__" == __name__:
    rospy.init_node("simulatedTgPosRecv", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("--server_ip", default="110.110.1.10")
    parser.add_argument("--server_port", default=60006)
    parser.add_argument("--gpsTopicName", default="/target/globalPosition")
    parser.add_argument("--rfConnectionTopicName", default="/GCS/from")
    args = parser.parse_args()
    posClient = PosClient(args.server_ip, args.server_port,
                          args.gpsTopicName, args.rfConnectionTopicName)
    posClient.run()
