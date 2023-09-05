from ConnectionInterface import ConnectionInterface
import os
from pymavlink import mavutil


class RfConnection(ConnectionInterface):
    def __init__(self, serialPort, baudRate):
        self.__inBuf = []
        self.__outBuf = []
        self.__serialPort = serialPort
        self.__baudRate = baudRate

        self.__initializePort()
        self.__receiveLoop()

        return

    def __initializePort(self):
        # Initialize Mavlink serial port
        os.environ["MAVLINK20"] = "1"
        self.__port = mavutil.mavlink_connection(
            self.__serialPort, baud=self.__baudRate, dialect="funnywing")
        return

    def __receiveLoop(self):
        pass

    def read(self):
        pass

    def write(self):
        pass
