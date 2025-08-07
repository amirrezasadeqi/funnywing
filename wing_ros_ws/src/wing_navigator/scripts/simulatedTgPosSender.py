#!/usr/bin/env python

import argparse
import select
import socket

import rospy
from pymavlink import mavutil

from wing_modules.Tcp_message import Tcp_message


class PosServer(object):
    def __init__(self, host, port, sitl_connection_string, posStreamFrequency):
        self._host = host
        self._port = port
        self._sitl_connection = mavutil.mavlink_connection(sitl_connection_string)
        self._sitl_connection.mav.request_data_stream_send(self._sitl_connection.target_system,
                                                           self._sitl_connection.target_component,
                                                           mavutil.mavlink.MAV_DATA_STREAM_POSITION, posStreamFrequency,
                                                           1)
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # This allows immediate reuse of the socket after closing the program.
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # To be able to close the program by keyboard interrupt when server is listening for the clients.
        self._server_socket.settimeout(5)
        return

    def start_server(self):
        self._server_socket.bind((self._host, self._port))
        rospy.loginfo(f"Server listening on {self._host}:{self._port}")
        while not rospy.is_shutdown():
            try:
                self._server_socket.listen()
            except socket.timeout:
                continue  # to prevent blocking by the listen function.
            except KeyboardInterrupt:
                rospy.loginfo("Server is shutting down.")
                self.stop_server()
                return
            # to not stuck in the while.
            ready_to_connect, _, _ = select.select([self._server_socket], [], [], 0)
            if ready_to_connect:
                break
        self._accept_connections()

    def stop_server(self):
        self._server_socket.close()

    def _accept_connections(self):
        while not rospy.is_shutdown():
            conn, addr = self._server_socket.accept()
            rospy.loginfo(f"Connected by {addr}")
            self._handle_client(conn)

    def _handle_client(self, conn):
        with conn:
            while not rospy.is_shutdown():
                try:
                    # receive data. recv_match is blocking, so there is no performance waste.
                    posMsg = self._sitl_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                    if posMsg and ("BAD_DATA" != posMsg.get_type()):
                        # create message and send it over the connection.
                        message = Tcp_message([posMsg.lat / 1.0e7, posMsg.lon / 1.0e7, posMsg.alt / 1.0e3])
                        message.send_over(conn)
                except Exception as e:
                    rospy.loginfo(f"Error handling handler: {e}")
                    break


if __name__ == "__main__":
    rospy.init_node("simulatedTgPosSender", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("--server_ip", default="110.110.1.10")
    parser.add_argument("--server_port", default=60006)
    parser.add_argument("--sitl_connection_string", default="tcp:localhost:5773")
    parser.add_argument("--pos_stream_freq", default=5)
    args, unknown = parser.parse_known_args()
    server = PosServer(args.server_ip, args.server_port, args.sitl_connection_string, args.pos_stream_freq)
    try:
        server.start_server()
    except KeyboardInterrupt:
        rospy.loginfo("Server is shutting down.")
        server.stop_server()
