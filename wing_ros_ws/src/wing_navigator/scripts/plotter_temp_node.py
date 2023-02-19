#!/usr/bin/env python

from numpy.core.defchararray import count
import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import pymap3d as pm
from wing_navigator.msg import GLOBAL_POSITION_INT
from itertools import count

lat0, lon0, alt0 = 35.41323864, 51.15932969, 1007


def wing_sub_callback(msg):
    latw, lonw, altw = pm.geodetic2enu(
        msg.gps_data.latitude, msg.gps_data.longitude, msg.gps_data.altitude, lat0, lon0, alt0)
    fw_lat.append(latw)
    fw_lon.append(lonw)
    fw_alt.append(altw)
    # rospy.loginfo(msg.latitude)


def target_sub_callback(msg):
    latt, lontt, altt = pm.geodetic2enu(
        msg.gps_data.latitude, msg.gps_data.longitude, msg.gps_data.altitude, lat0, lon0, alt0)
    ft_lat.append(latt)
    ft_lon.append(lontt)
    ft_alt.append(altt)


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
    # create the plot
    global fw_lat, fw_lon, fw_alt
    fw_lat, fw_lon, fw_alt = [], [], []
    global ft_lat, ft_lon, ft_alt
    ft_lat, ft_lon, ft_alt = [], [], []
    rospy.Subscriber("/wing_gps_topic_gcs",
                     GLOBAL_POSITION_INT, wing_sub_callback)
    rospy.Subscriber("/target_gps_topic", GLOBAL_POSITION_INT,
                     target_sub_callback)
    #plotter_thread = threading.Thread(target= publish)
    # plotter_thread.start()
    # plotter3D()
    # plotter2D()
    plot_path()
    rospy.spin()
