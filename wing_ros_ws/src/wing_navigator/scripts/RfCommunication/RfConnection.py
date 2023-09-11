import os
import threading
import time
import rospy
from pymavlink import mavutil
from ConnectionInterface import ConnectionInterface


# TODO: check if singleton pattern is suitable for this connection
class RfConnection(ConnectionInterface):
    def __init__(self, serialPort="/dev/ttyUSB0", baudRate=9600, outBufWaitForMsg=1e-4):
        self.__outBufWaitForMsg = outBufWaitForMsg
        self.__inBuf = []
        self.__outBuf = []
        self.__serialPort = serialPort
        self.__baudRate = baudRate

        self.__startCommunication()

        return

    def read(self):
        if len(self.__inBuf):
            return self.__inBuf.pop(0)
        else:
            return None

    def write(self, message):
        self.__outBuf.append(message)

    def getPort(self):
        return self.__port

    def __startCommunication(self):
        self.__initializePort()
        # Initialize threads for read/write and ... tasks
        self.__recvThread = threading.Thread(target=self.__recvLoop)
        self.__sendThread = threading.Thread(target=self.__sendLoop)
        self.__recvThread.start()
        self.__sendThread.start()
        return

    def __initializePort(self):
        # Initialize Mavlink serial port
        os.environ["MAVLINK20"] = "1"
        self.__port = mavutil.mavlink_connection(
            self.__serialPort, baud=self.__baudRate, dialect="funnywing")
        return

    def __recvLoop(self):
        while not rospy.is_shutdown():
            inMsg = self.__port.recv_match(blocking=True, timeout=1.0)
            if inMsg and ("BAD_DATA" != inMsg.get_type()):
                self.__inBuf.append(inMsg)
        return

    def __sendLoop(self):
        while not rospy.is_shutdown():
            if len(self.__outBuf):
                outMsg = self.__outBuf.pop(0)
                self.__port.mav.send(outMsg)
            else:
                time.sleep(self.outBufWaitForMsg)
        return

    def __del__(self):
        # TODO: check if we need to join the running threads here or not.
        self.__port.close()
