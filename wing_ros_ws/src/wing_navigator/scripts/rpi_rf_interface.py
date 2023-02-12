#!/usr/bin/env python3

import rospy
import threading
import argparse
from pymavlink import mavutil, mavwp
import os
from wing_navigator.srv import ActiveModeRequest, SimpleGotoRequest
from wing_modules.navigator_modules.navigator_client import navigator_client

mode_mapping = {
    1: "MANUAL",
    2: "AUTO",
    3: "GUIDED",
    4: "TAKEOFF",
    5: "LOITER",
    6: "RTL",
    7: "FBWB",
    8: "CIRCLE",
    9: "FBWA"
}


def reader_worker(port, input_msgs):
    '''
        Just reads the input messages or commands as fast as possible, no data processing
        to prevent data loss.
    '''
    while not rospy.is_shutdown():
        input_msgs.append(port.recv_match(blocking=True, timeout=1.0))


def cmd_handler_worker(cmds, cmd_anss):
    '''
        Getting the command from command list and after checking the correctness
        and command type, handles the request and getting of the answer and adds
        the answer to the command answer list to send the answers by another thread
        to the GCS.
    '''
    # client object to send requests to navigator server
    fw_client = navigator_client("wing")

    while not rospy.is_shutdown():
        if len(cmds) != 0:
            cmd = cmds.pop(0)
            if not cmd:
                continue
            elif cmd.get_type() == "BAD_DATA":
                continue
            else:
                # correct commands arive here and must be handled here
                cmd_anss.append(cmd_handle(cmd, fw_client))

        else:
            continue


def cmd_handle(cmd, client_obj):
    # check the command type
    # create and fill the request object
    # call the proper request from the object
    # return response of the request
    if cmd.get_type() == "COMMAND_LONG":
        cmd_as_dict = cmd.to_dict()
        if cmd_as_dict['command'] == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            mode_number = cmd_as_dict["param2"]
            req = ActiveModeRequest(mode_mapping[mode_number])
            resp = {"type": "MAV_CMD_DO_SET_MODE", "sequence": None,
                    "response": client_obj.active_mode_client(req)}
            return resp
        elif cmd_as_dict['command'] == mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO:
            req = SimpleGotoRequest()
            req.lat = cmd_as_dict['param5']
            req.lon = cmd_as_dict['param6']
            req.alt = cmd_as_dict['param7']
            resp = {"type": "MAV_CMD_OVERRIDE_GOTO", "sequence": None,
                    "response": client_obj.simple_goto_client(req)}
            return resp
        else:
            rospy.loginfo(
                f"The command type {cmd.get_type()} is not supported yet!")
            resp = {"type": None, "sequence": None, "response": None}
            return resp
    else:
        rospy.loginfo(
            f"The command type {cmd.get_type()} is not supported yet!")
        resp = {"type": None, "sequence": None, "response": None}
        return resp


def cmd_resp_sender_worker(port, cmd_anss):

    while not rospy.is_shutdown():
        if len(cmd_anss) != 0:
            ans = cmd_anss.pop(0)
            if ans['type'] == None:
                continue
            else:
                send_cmd_resp(port, ans)
        else:
            continue


def send_cmd_resp(port, ans):
    if ans['type'] == "MAV_CMD_DO_SET_MODE":
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED if ans[
            'response'] else mavutil.mavlink.MAV_RESULT_FAILED
        port.mav.command_ack_send(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, result, 0, 0, 0, 0)
    elif ans['type'] == "MAV_CMD_OVERRIDE_GOTO":
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED if ans[
            'response'] else mavutil.mavlink.MAV_RESULT_FAILED
        port.mav.command_ack_send(
            mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO, result, 0, 0, 0, 0)
    else:
        rospy.loginfo(f"The command type {ans['type']} is not supported yet!")


def sensor_sender_worker(port):
    pass


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

    # In future you may need some information other than commands from GCS
    # input_msgs = []
    # You may also have a list containing all the output messages, e.g. sensor data
    # , command answers and so on. You may also have multi-threads for sending each
    # type because as I know and stated in the obsidian vault of Telem-communication
    # writing to mavlink_connection is thread-safe.
    # output_msgs = []
    cmds = []
    cmd_anss = []

    reader_thread = threading.Thread(
        target=reader_worker, args=(port, cmds))
    cmd_handler_thread = threading.Thread(
        target=cmd_handler_worker, args=(cmds, cmd_anss))
    cmd_resp_sender_thread = threading.Thread(
        target=cmd_resp_sender_worker, args=(port, cmd_anss))
    sensor_sender_thread = threading.Thread(
        target=sensor_sender_worker, args=(port, ))

    reader_thread.start()
    cmd_handler_thread.start()
    cmd_resp_sender_thread.start()
    sensor_sender_thread.start()
