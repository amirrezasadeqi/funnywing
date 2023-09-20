#!/usr/bin/env python

import argparse
from itertools import count

import matplotlib.pyplot as plt
import pymap3d as pm
import rospy
from matplotlib.animation import FuncAnimation
from numpy.core.defchararray import count
from wing_navigator.msg import GLOBAL_POSITION_INT
from wing_navigator.srv import MSL_WGS_CONVRequest, MSL_WGS_CONV

lat0, lon0, alt0 = 35.41323864, 51.15932969, 1007


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
        print("Service Call Failed: %s" % e)


def wing_sub_callback(msg):
    lat = msg.gps_data.latitude
    lon = msg.gps_data.longitude
    alt = msg.gps_data.altitude  # height relative to MSL
    wgs_alt = msl2ellipsoid(lat, lon, alt)
    x, y, z = pm.geodetic2ecef(lat, lon, wgs_alt)
    fw_lat.append(x)
    fw_lon.append(y)
    fw_alt.append(z)
    # rospy.loginfo(msg.latitude)


def target_sub_callback(msg):
    lat = msg.gps_data.latitude
    lon = msg.gps_data.longitude
    alt = msg.gps_data.altitude  # height relative to MSL
    wgs_alt = msl2ellipsoid(lat, lon, alt)
    x, y, z = pm.geodetic2ecef(lat, lon, wgs_alt)
    ft_lat.append(x)
    ft_lon.append(y)
    ft_alt.append(z)


def plot_path():
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    plt.style.use('fivethirtyeight')

    index = count()

    def animate(i):
        plt.cla()
        plt.plot(fw_lat, fw_lon, fw_alt, label='Funnywing')
        plt.plot(ft_lat, ft_lon, ft_alt, label='target')
        plt.legend(loc='upper left')
        plt.tight_layout()

    ani = FuncAnimation(plt.gcf(), animate, interval=1000)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    rospy.init_node("plotte_node")

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--simulation", default=False)
    args = parser.parse_args()

    # create the plot
    global fw_lat, fw_lon, fw_alt
    fw_lat, fw_lon, fw_alt = [], [], []
    global ft_lat, ft_lon, ft_alt
    ft_lat, ft_lon, ft_alt = [], [], []
    wing_gps_topic_name = "/wing_gps_topic" if args.simulation else "/wing_gps_topic_gcs"
    rospy.Subscriber(wing_gps_topic_name,
                     GLOBAL_POSITION_INT, wing_sub_callback)
    rospy.Subscriber("/target_gps_topic", GLOBAL_POSITION_INT,
                     target_sub_callback)
    # plotter_thread = threading.Thread(target= publish)
    # plotter_thread.start()
    # plotter3D()
    # plotter2D()
    plot_path()
    rospy.spin()
