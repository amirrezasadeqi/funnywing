#!/usr/bin/env python3


import rospy
import serial
from wing_navigator.msg import GLOBAL_POSITION_INT


if __name__ == "__main__":
    rospy.init_node("target_gps_reciever")

    port = serial.Serial("/dev/ttyUSB0", 9600, timeout=1.0)
    if not port.is_open:
        port.open()

    r = rospy.Rate(4)
    pub = rospy.Publisher("/target_gps_topic",
                          GLOBAL_POSITION_INT, queue_size=1)
    msg = GLOBAL_POSITION_INT()
    while not rospy.is_shutdown():
        try:
            recv_msg = port.readline().decode('utf-8')
        except:
            continue

        if not recv_msg.startswith("$GNGLL"):
            continue
        else:
            try:
                recv_msg = recv_msg.split(",")
                msg.gps_data.latitude = float(recv_msg[1]) / 100.0
                msg.gps_data.longitude = float(recv_msg[3]) / 100.0
                msg.gps_data.altitude = float(recv_msg[5])
                pub.publish(msg)
                # print(
                #     f"lat: {float(hello[1]) / 100.0}, lon: {float(hello[3]) / 100.0}, alt: {float(hello[5])}")
            except:
                print("Data is corrupted!")
                continue
        r.sleep()
