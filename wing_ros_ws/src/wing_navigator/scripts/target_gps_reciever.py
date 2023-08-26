#!/usr/bin/env python3


import rospy
import serial
from wing_navigator.msg import GLOBAL_POSITION_INT
import argparse


def parse_gngll_data(msg_as_list):
    '''
    converts the GPGLL/GNGLL data format to the degrees
    :param msg_as_list: A list containing GPS data fields
    :return: latitude, longitude and altitude in degrees
    '''
    lat_str = msg_as_list[2]
    lon_str = msg_as_list[4]
    alt_str = msg_as_list[9]  # height relative to MSL or geoid
    # convert lat_str to lat in degrees DDMM.MMMMM
    lat = float(lat_str[:2]) + (float(lat_str[2:]) / 60.0)
    # convert lon_str to lon in degrees DDDMM.MMMMM
    lon = float(lon_str[:3]) + (float(lon_str[3:]) / 60.0)
    alt = float(alt_str)
    return lat, lon, alt


if __name__ == "__main__":
    rospy.init_node("target_gps_reciever")

    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB1")
    parser.add_argument("-b", "--baudrate", default=9600)
    args = parser.parse_args()

    # AT COMMAND to set the gps rate to 5Hz: 0xC8 0x00 -> 200[ms] I think!
    ubx_com = b'\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A\xB5\x62\x06\x08\x00\x00\x0E\x30'
    port = serial.Serial(args.serial_port, args.baudrate, timeout=1.0)
    if not port.is_open:
        port.open()

    # Sending command to ublox module to change the measurement rate. This change is not permanent and Restarting
    # the module resets the configs.
    port.write(ubx_com)

    # TODO: check if you should use (measurement rate) * (number of gps data line) for the rate or not!
    r = rospy.Rate(25)

    pub = rospy.Publisher("/target_gps_topic",
                          GLOBAL_POSITION_INT, queue_size=1)
    msg = GLOBAL_POSITION_INT()
    while not rospy.is_shutdown():
        try:
            recv_msg = port.readline().decode('utf-8')
        except:
            continue

        # if not recv_msg.startswith("$GNGLL"):
        if not recv_msg.startswith("$GNGGA"):
            continue
        else:
            try:
                recv_msg = recv_msg.split(",")
                ###########################################
                lat, lon, alt = parse_gngll_data(recv_msg)
                msg.gps_data.latitude = lat
                msg.gps_data.longitude = lon
                msg.gps_data.altitude = alt
                ###########################################
                # msg.gps_data.latitude = float(recv_msg[1]) / 100.0
                # msg.gps_data.longitude = float(recv_msg[3]) / 100.0
                # msg.gps_data.altitude = float(recv_msg[5])
                pub.publish(msg)
                # print(
                #     f"lat: {float(hello[1]) / 100.0}, lon: {float(hello[3]) / 100.0}, alt: {float(hello[5])}")
            except:
                print("Data is corrupted!")
                continue
        r.sleep()
