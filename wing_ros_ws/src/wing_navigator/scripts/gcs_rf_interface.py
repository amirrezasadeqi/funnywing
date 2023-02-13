#!/usr/bin/env python3

from distutils import cmd
import rospy
import threading
import msgpack
import time
import argparse
import os
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header, String, UInt8MultiArray
from wing_navigator.msg import GLOBAL_POSITION_INT
from wing_modules.navigator_modules.TimerAP import TimerAP


def reader_worker(port, input_msgs):
    '''
        Just reads the input as fast as possible, no data processing to prevent
        data loss.
    '''
    while not rospy.is_shutdown():
        input_msgs.append(port.recv_match(blocking=True, timeout=1.0))


def pub_worker(input_msgs):

    gps_pub = rospy.Publisher(
        "/wing_gps_topic_gcs", GLOBAL_POSITION_INT, queue_size=1)

    # To prevent definition of new ROS messages we encode data to bytes
    # using msgpack and publish them with UInt8MultiArray type.
    cmd_resp_pub = rospy.Publisher(
        "/wing_cmd_resp_topic_gcs", UInt8MultiArray, queue_size=1)

    while not rospy.is_shutdown():
        if len(input_msgs) != 0:
            msg = input_msgs.pop(0)
            if not msg:
                continue
            elif msg.get_type() == "BAD_DATA":
                continue
            else:
                publisher = gps_pub if msg.get_type() == "GLOBAL_POSITION_INT" else cmd_resp_pub
                ros_msg = mav_to_ros_msg(msg)
                if ros_msg is not None:
                    publisher.publish(ros_msg)
        else:
            continue


def mav_to_ros_msg(msg):
    '''converts mavlink message into ros message'''
    if msg.get_type() == "GLOBAL_POSITION_INT":
        # convert mavlink msg to GLOBAL_POSITION_INT.msg type
        ros_msg = GLOBAL_POSITION_INT()
        ros_msg.gps_data = NavSatFix()
        ros_msg.gps_data.header = Header()
        ros_msg.gps_data.header.stamp = rospy.Time.from_sec(
            msg.time_boot_ms / 1000.0)
        ros_msg.gps_data.header.frame_id = 'gps'
        # The lat and long are scaled due to low resolution of floats, so descale them.
        ros_msg.gps_data.latitude = msg.lat / 1.0e7
        ros_msg.gps_data.longitude = msg.lon / 1.0e7
        ros_msg.gps_data.altitude = msg.relative_alt / 1000.0
        ros_msg.gps_data.status.status = NavSatStatus.STATUS_SBAS_FIX
        ros_msg.gps_data.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        # GLOBAL_POSITION_INT message velocities are in cm/sec
        ros_msg.velocity.x = msg.vx / 100.0
        ros_msg.velocity.y = msg.vy / 100.0
        ros_msg.velocity.z = msg.vz / 100.0

        # convert msg to NavSatFix
        # ros_msg = NavSatFix()
        # ros_msg.header = Header()
        # ros_msg.header.stamp = rospy.Time.from_sec(msg.time_boot_ms / 1000.0)
        # ros_msg.header.frame_id = 'gps'
        # # The lat and long are scaled due to low resolution of floats, so descale them.
        # ros_msg.latitude = msg.lat / 1.0e7
        # ros_msg.longitude = msg.lon / 1.0e7
        # ros_msg.altitude = msg.relative_alt / 1000.0
        # ros_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        # ros_msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO

    elif msg.get_type() == "COMMAND_RESPONSE":
        # Convert msg to a ROS type to publish it
        # You can use enums "MAV_RESULT_ACCEPTED/FAILED" for simple pass and fail
        ros_msg = UInt8MultiArray()
        ros_msg.data = msgpack.packb(msg)
    else:
        ros_msg = None

    return ros_msg


class cmd_subscriber_and_sender:
    """
    A class containing the subscriber object listening to the command topic
    and on callback converts ROS messages to mavlink messages and sends them 
    to the RPI.
    """

    def __init__(self, name, port):
        self._name = name
        self._port = port
        self._subscriber = rospy.Subscriber(
            "/wing_nav_cmds_gcs", UInt8MultiArray, self._callback)

    def _callback(self, msg):
        cmd_obj = msgpack.unpackb(msg.data)
        self._send_mav_cmd(cmd_obj)

    def _send_mav_cmd(self, cmd_obj):
        # TODO: use the wraper to automize calling the sender function.
        cmd_type = cmd_obj['command_type']
        if cmd_type == "active_mode":
            self._port.mav.command_long_send(
                0, 0, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_GUIDED_ARMED, cmd_obj["flight_mode_number"], 0, 0, 0, 0, 0)
        elif cmd_type == "simple_goto":
            x, y, z = cmd_obj['lat'], cmd_obj['lon'], cmd_obj['alt']
            self._port.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO, 0,
                                             mavutil.mavlink.MAV_GOTO_DO_HOLD, mavutil.mavlink.MAV_GOTO_HOLD_AT_SPECIFIED_POSITION, 0, 0, x, y, z)
        else:
            rospy.loginfo(
                f"The command type {cmd_obj['command_type']} is not supported yet!")

    def inform(self):
        rospy.loginfo(
            f"{self._name} is listening to GUI commands and sends them to RPI!")


def cmd_sub_and_send_worker(port):
    cmdSubcriberSender = cmd_subscriber_and_sender("CmdSubSender", port)
    cmdSubcriberSender.inform()
    rospy.spin()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--system")
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", default=57600)
    args = parser.parse_args()

    rospy.init_node(f"{args.system}_rf_interface")

    os.environ["MAVLINK20"] = "1"
    port = mavutil.mavlink_connection(
        args.serial_port, baud=args.baudrate, dialect="funnywing")

    input_msgs = []

    reader_thread = threading.Thread(
        target=reader_worker, args=(port, input_msgs))
    publisher_thread = threading.Thread(
        target=pub_worker, args=(input_msgs, ))
    cmd_sub_and_send_thread = threading.Thread(
        target=cmd_sub_and_send_worker, args=(port, ))

    reader_thread.start()
    publisher_thread.start()
    cmd_sub_and_send_thread.start()
