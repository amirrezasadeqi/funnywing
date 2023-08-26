#!/usr/bin/env python3

import rospy
import msgpack
from std_msgs.msg import UInt8MultiArray
from wing_navigator.msg import GLOBAL_POSITION_INT
from wing_navigator.srv import MSL_WGS_CONVRequest, MSL_WGS_CONV
from wing_modules.navigator_modules.navigation_commands import navigation_commands as nav_com
import pymap3d as pm
import numpy as np

global last_fw_global_pos
last_fw_global_pos = [35.41323864, 51.15932969, 1007]

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

def ellipsoid2msl(wgs_lat, wgs_lon, wgs_alt):
    req = MSL_WGS_CONVRequest()
    req.msl_in = False
    req.lat = wgs_lat
    req.lon = wgs_lon
    req.alt = wgs_alt
    server_name = "/msl_wgs_conv_service"
    rospy.wait_for_service(server_name)
    try:
        msl_wgs_conv_client = rospy.ServiceProxy(server_name, MSL_WGS_CONV)
        resp = msl_wgs_conv_client(req)
        msl_alt = resp.alt_convd
        return msl_alt
    except rospy.ServiceException as e:
        print("Service Call Failed: %s"%e)

def real2virt_target_pos_converter(tg_global_lat, tg_global_lon, tg_global_alt, fw_lat, fw_lon, fw_alt, offset):
    '''
    Gives a virtual target ahead of the funnywing to track. GPS data converted to ECEF coordinate and after computations
    converted back to the WGS84 coordinate frame
    :param tg_global_lat:
    :param tg_global_lon:
    :param tg_global_alt: target hei
    :param fw_lat:
    :param fw_lon:
    :param fw_alt: funnywing height relative to MSL.
    :param offset: offset ahead of the funnywing
    :return: Virtual target position in WGS84. Altitude is relative to MSL or geoid
    '''
    # lat0, lon0, alt0 = 35.41323864, 51.15932969, 1007
    # tg_local_pos = pm.geodetic2enu(
    #     tg_global_pos, tg_global_lon, tg_global_alt, lat0, lon0, alt0)
    # fw_local_pos = pm.geodetic2enu(
    #     fw_lat, fw_lon, fw_alt, lat0, lon0, alt0)

    # TODO: convert altitude from MSL to above WGS84 ellipsoid for using it in the geodetic2ecef function
    # tg_local_pos = pm.geodetic2ecef(tg_global_lat, tg_global_lon, tg_global_alt)
    # fw_local_pos = pm.geodetic2ecef(fw_lat, fw_lon, fw_alt)
    tg_global_alt_ellipsoid = msl2ellipsoid(tg_global_lat, tg_global_lon, tg_global_alt)
    fw_alt_ellipsoid = msl2ellipsoid(fw_lat, fw_lon, fw_alt)
    tg_local_pos = pm.geodetic2ecef(tg_global_lat, tg_global_lon, tg_global_alt_ellipsoid)
    fw_local_pos = pm.geodetic2ecef(fw_lat, fw_lon, fw_alt_ellipsoid)

    diff_vec = np.array(tg_local_pos) - np.array(fw_local_pos)
    diff_vec_norm = np.linalg.norm(diff_vec)
    diff_unit_vec = diff_vec / diff_vec_norm
    ##################################################################
    # Print the Distance to target for test purposes
    print(f"Distance to Target: {diff_vec_norm}")
    ##################################################################
    # virt_tg_local_pos = tg_local_pos + offset * diff_unit_vec
    virt_tg_local_pos = fw_local_pos + offset * diff_unit_vec
    # virt_tg_global_pos = pm.enu2geodetic(
    #     virt_tg_local_pos[0], virt_tg_local_pos[1], virt_tg_local_pos[2], lat0, lon0, alt0)
    virt_tg_global_pos = pm.ecef2geodetic(virt_tg_local_pos[0], virt_tg_local_pos[1], virt_tg_local_pos[2])
    # TODO: ecef2geodetic gives altitude above WGS84 ellipsoid. To be compatible with DroneKit convert it back to
    #       Altitude relative to MSL or geoid
    virt_tg_lat = virt_tg_global_pos[0]
    virt_tg_lon = virt_tg_global_pos[1]
    # virt_tg_alt = virt_tg_global_pos[2]
    virt_tg_alt = ellipsoid2msl(virt_tg_lat, virt_tg_lon, virt_tg_global_pos[2])
    return virt_tg_lat, virt_tg_lon, virt_tg_alt


def simple_tracker_cb(msg, args):
    # I think this line solves the problem of not updating the funnywing position
    global last_fw_global_pos
    if last_fw_global_pos is not None:
        publisher = args[0]
        # real target gps location
        tg_lat = msg.gps_data.latitude
        tg_lon = msg.gps_data.longitude
        tg_alt = msg.gps_data.altitude
        fw_lat, fw_lon, fw_alt = last_fw_global_pos[0], last_fw_global_pos[1], last_fw_global_pos[2]
        # TODO: you should tune the last argument for your funnywing and altitude should be with respect to the home altitude.
        virt_tg_lat, virt_tg_lon, virt_tg_alt = real2virt_target_pos_converter(
            tg_lat, tg_lon, tg_alt, fw_lat, fw_lon, fw_alt, 120)
        cmd = nav_com.simple_goto(virt_tg_lat, virt_tg_lon, virt_tg_alt)
        packed_cmd = msgpack.packb(cmd)
        cmd_msg = UInt8MultiArray()
        cmd_msg.data = packed_cmd
        publisher.publish(cmd_msg)
    else:
        rospy.loginfo("The funnywing gps is not subscribed yet!")


def fw_gps_sub_handler(msg):
    global last_fw_global_pos
    last_fw_global_pos = [msg.gps_data.latitude,
                          msg.gps_data.longitude, msg.gps_data.altitude]


if __name__ == "__main__":
    rospy.init_node("simple_tracker_node")
    rospy.loginfo("Starting simple_tracker_node.")

    publisher = rospy.Publisher(
        "/wing_nav_cmds_gcs", UInt8MultiArray, queue_size=1)
    tg_gps_subscriber = rospy.Subscriber(
        "/target_gps_topic", GLOBAL_POSITION_INT, simple_tracker_cb, (publisher, ))
    fw_gps_subscriber = rospy.Subscriber(
        "/wing_gps_topic_gcs", GLOBAL_POSITION_INT, fw_gps_sub_handler)

    rospy.spin()
