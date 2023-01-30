#!/usr/bin/env python3

import rospy
import json
import threading
import time
import serial
import argparse


class test_command:

    def __init__(self, system, logfile):
        self._system = system
        self._logfile = logfile
        self.reader_objs_to_log = []
        self.writer_logs = []

    @staticmethod
    def test_message(sen: str, seq: int) -> dict:
        return {
            "command_type": "hello_world",
            "content": f"{seq}: Hello from {sen} writer thread!",
            "sender": sen,
            "sequence": seq
        }

    @staticmethod
    def message_encoder(msg_obj) -> bytes:
        msg = json.dumps(msg_obj)
        msg += "\n"
        msg = msg.encode('utf-8')
        return msg

    @staticmethod
    def port_to_str(port) -> str:
        # check the case inwhich there is no data on serial port input
        return port.readline().decode('utf-8').rstrip("\n")

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
                        f.write(f"{log['content']}\n")
            else:
                print("row parameter can be 'r' or 'w' and nothing else!")

        return


def writer_worker(port, test_command_obj):
    wc = 0
    if not port.isOpen():
        port.open()
    while not rospy.is_shutdown():
        rospy.loginfo(f"{test_command_obj.system} writer thread writes!")
        msg_obj = test_command.test_message(f"{test_command_obj.system}", wc)
        msg = test_command.message_encoder(msg_obj)
        port.write(msg)
        wc += 1
        time.sleep(0.5)

    test_command_obj.writer_logs.append(wc)

    return


def reader_worker(port, test_command_obj):

    if not port.isOpen():
        port.open()

    while not rospy.is_shutdown():
        msg_str = test_command.port_to_str(port)
        if len(msg_str) == 0:
            continue
        try:
            msg_obj = json.loads(msg_str)
            test_command_obj.reader_objs_to_log.append(msg_obj)
        except:
            test_command_obj.reader_objs_to_log.append(
                "Bad Date received and can't be decoded!")

    return


if __name__ == "__main__":
    rospy.init_node("gcs_rftester_node")

    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output")
    parser.add_argument("-s", "--system")
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", default=57600)

    args = parser.parse_args()

    port = serial.Serial(args.serial_port, baudrate=args.baudrate, timeout=1.0)

    test_command_obj = test_command(args.system, args.output)

    writer_thread = threading.Thread(
        target=writer_worker, args=(port, test_command_obj))
    reader_thread = threading.Thread(
        target=reader_worker, args=(port, test_command_obj))

    writer_thread.start()
    reader_thread.start()

    writer_thread.join()
    test_command_obj.log_test_results("w")
    reader_thread.join()
    test_command_obj.log_test_results("r")
