#!/usr/bin/env python3

import rospy
import threading
import time
import argparse
from pymavlink import mavutil
import os


class data_logger:

    def __init__(self, system, logfile):
        self._system = system
        self._logfile = logfile
        self.reader_objs_to_log = []
        self.writer_logs = []

    def log_test_results(self, row):
        with open(self._logfile, "a") as f:
            if (row == "w"):
                f.write(f"{self._system} writer thread logs:\n")
                f.write(
                    f"Number of messages have been sent by {self._system} are: {self.writer_logs.pop()}\n\n")
            elif (row == "r"):
                f.write(f"{self._system} reader thread logs:\n")
                for log in self.reader_objs_to_log:
                    if isinstance(log, str):
                        f.write(f"{log}\n")
                    else:
                        f.write(
                            f"{self._system} reader thread reads: (SENDER_ID: {log.sender_id}, SENDER_NAME: {log.sender_name}, SEQUENCE: {log.sequence})\n")
            else:
                print("row parameter can be 'r' or 'w' and nothing else!")

        return


def writer_worker(port, data_logger_obj):
    wc = 0

    while not rospy.is_shutdown():
        rospy.loginfo(f"{data_logger_obj._system} writer thread writes!")
        sender_id = mavutil.mavlink.SENDER_ID_GCS if data_logger_obj._system == "GCS" else mavutil.mavlink.SENDER_ID_RPI
        port.mav.hello_world_send(
            sender_id, f"{data_logger_obj._system}".encode('utf-8'), wc)
        wc += 1
        time.sleep(0.5)

    data_logger_obj.writer_logs.append(wc)

    return


def reader_worker(port, data_logger_obj):
    sender = "RPI" if data_logger_obj._system == "GCS" else "GCS"
    while not rospy.is_shutdown():
        msg = port.recv_match(blocking=True, timeout=1.0)
        if not msg:
            continue
        elif msg.get_type() == "BAD_DATA":
            log_msg = f"Bad Data received from {sender} and "
            if mavutil.all_printable(msg.data):
                log_msg += f"is printable like:\n{msg.data}"
            else:
                log_msg += "is not printable!"

            data_logger_obj.reader_objs_to_log.append(log_msg)
        else:
            # Valid message
            data_logger_obj.reader_objs_to_log.append(msg)

    return


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output")
    parser.add_argument("-s", "--system")
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", default=57600)

    args = parser.parse_args()

    rospy.init_node(f"{args.system}_rftester_node")

    # setting the mavlink version when using mavutil for connection management
    os.environ["MAVLINK20"] = "1"  # 1:MAVLINK20, 0:MAVLINK10
    # Using funnywing Dialect which includes ArduPilot messages
    port = mavutil.mavlink_connection(
        args.serial_port, baud=args.baudrate, dialect="funnywing")

    data_logger_obj = data_logger(args.system, args.output)

    writer_thread = threading.Thread(
        target=writer_worker, args=(port, data_logger_obj))
    reader_thread = threading.Thread(
        target=reader_worker, args=(port, data_logger_obj))

    writer_thread.start()
    reader_thread.start()

    writer_thread.join()
    data_logger_obj.log_test_results("w")
    reader_thread.join()
    data_logger_obj.log_test_results("r")
