import os
import threading
import time
import rospy
from pymavlink import mavutil
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface


# TODO: check if singleton pattern is suitable for this connection
class RfConnection(ConnectionInterface):
    def __init__(self, serialPort="/dev/ttyUSB0", baudRate=9600, outBufWaitForMsg=1e-4):
        self._outBufWaitForMsg = outBufWaitForMsg
        self._inBuf = []
        self._outBuf = []
        self._serialPort = serialPort
        self._baudRate = baudRate
        self._port = None

        self._startCommunication()

        return

    def read(self):
        if len(self._inBuf):
            return self._inBuf.pop(0)
        else:
            return None

    def write(self, message):
        self._outBuf.append(message)

    def getPort(self):
        return self._port

    def _startCommunication(self):
        self._initializePort()
        # Initialize threads for read/write and ... tasks
        self._recvThread = threading.Thread(target=self._recvLoop)
        self._sendThread = threading.Thread(target=self._sendLoop)
        self._recvThread.start()
        self._sendThread.start()
        return

    def _initializePort(self):
        # Initialize Mavlink serial port
        os.environ["MAVLINK20"] = "1"
        try:
            self._port = mavutil.mavlink_connection(
                self._serialPort, baud=self._baudRate, dialect="funnywing")
        except:
            self._port = None
            print("The Connection is Not initialized Correctly!")
        return

    def _recvLoop(self):
        while not rospy.is_shutdown():
            inMsg = self._port.recv_match(blocking=True, timeout=1.0)
            if inMsg and ("BAD_DATA" != inMsg.get_type()):
                self._inBuf.append(inMsg)
        return

    def _sendLoop(self):
        while not rospy.is_shutdown():
            if len(self._outBuf):
                outMsg = self._outBuf.pop(0)
                self._port.mav.send(outMsg)
            else:
                time.sleep(self._outBufWaitForMsg)
        return

    def __del__(self):
        # TODO: check if we need to join the running threads here or not.
        self._port.close()
