#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import argparse
from mpl_toolkits import mplot3d
from wing_navigator.msg import GLOBAL_POSITION_INT
import pymap3d as pm
# import numpy as np
# import threading
from matplotlib.animation import FuncAnimation
from wing_navigator.srv import MSL_WGS_CONVRequest, MSL_WGS_CONV


def msl2ellipsoid(msl_lat, msl_lon, msl_alt):
    req = MSL_WGS_CONVRequest()
    req.msl_in = True
    req.lat = msl_lat
    req.lon = msl_lon
    req.alt = msl_alt
    server_name = "/msl_wgs_conv_service"
    rospy.wait_for_service(server_name)
    try:
        msl_wgs_conv_client = rospy.ServiceProxy(server_name, MSL_WGS_CONV)
        resp = msl_wgs_conv_client(req)
        wgs_alt = resp.alt_convd
        return wgs_alt
    except rospy.ServiceException as e:
        print("Service Call Failed: %s"%e)

class path_plotter():

    def __init__(self, simulation=False):
        self.tg_data = []
        self.fw_data = []
        self.lat0, self.lon0, self.alt0 = 35.41323864, 51.15932969, 1007
        # self.fig = plt.figure(figsize=(6, 6))
        # self.ax = plt.axes(projection='3d')
        # self.ax = plt.axes()
        # plt.show(block=False)
        self.tg_gps_subscriber = rospy.Subscriber(
            "/target_gps_topic", GLOBAL_POSITION_INT, self.tg_path_plotter_cb)
        wing_gps_topic_name = "/wing_gps_topic" if simulation else "/wing_gps_topic_gcs"
        self.fw_gps_subscriber = rospy.Subscriber(
            wing_gps_topic_name, GLOBAL_POSITION_INT, self.fw_path_plotter_cb)
        # self.plotter_thread = threading.Thread(target=self.plotter_worker)
        # self.plotter_thread.start()

    # def plotter_worker(self):
    #     while not rospy.is_shutdown():
    #         self

    def plot(self):
        x_data = [row[0] for row in self.fw_data]
        y_data = [row[1] for row in self.fw_data]
        x_data2 = [row[0] for row in self.tg_data]
        y_data2 = [row[1] for row in self.tg_data]
        figure = plt.figure()
        line, = plt.plot(x_data, y_data, '-')
        line2, = plt.plot(x_data2, y_data2, '-')

        def update(frame):
            line.set_data(x_data, y_data)
            line2.set_data(x_data2, y_data2)
            figure.gca().relim()
            figure.gca().autoscale_view()
            return line,

        animation = FuncAnimation(figure, update, interval=200)
        plt.show()

    def tg_path_plotter_cb(self, msg):
        lat = msg.gps_data.latitude
        lon = msg.gps_data.longitude
        alt = msg.gps_data.altitude  # height with respect to MSL
        wgs_alt = msl2ellipsoid(lat, lon, alt)
        tg_local_pos = pm.geodetic2ecef(lat, lon, wgs_alt)
        self.tg_data.append(tg_local_pos)

    def fw_path_plotter_cb(self, msg):
        lat = msg.gps_data.latitude
        lon = msg.gps_data.longitude
        alt = msg.gps_data.altitude  # height with respect to MSL
        wgs_alt = msl2ellipsoid(lat, lon, alt)
        fw_local_pos = pm.geodetic2ecef(lat, lon, wgs_alt)
        self.fw_data.append(fw_local_pos)


if __name__ == "__main__":
    rospy.init_node("path_plotter_node")
    rospy.loginfo("Starting path_plotter_node.")
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--simulation", default=False)
    args = parser.parse_args()

    plotter_object = path_plotter(args.simulation)
    plotter_object.plot()

    rospy.spin()
