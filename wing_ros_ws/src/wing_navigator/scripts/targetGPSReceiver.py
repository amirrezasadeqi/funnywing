#!/usr/bin/env python

import argparse
import threading

import rospy
import serial
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix


class GPSDataReceiver(object):
    def __init__(self, portAddress, baudRate, gpsTopicName, rfConnectionTopicName, gpsDataRate):
        self._port = None
        self._initializePort(portAddress, baudRate)
        self._configUbloxModule()
        # Creating MAVLink object to construct Mavlink message from mavlink one. Note that the code is running on GCS
        # so the sender or source system is GCS.
        self._mavProtocolObj = mavutil.mavlink.MAVLink("", mavutil.mavlink.MAV_TYPE_GCS, 1)

        # TODO: check if you should use (measurement rate) * (number of gps data line) for the rate or not!
        # Each GPS data advertised in 5 different GPS formats and each of formats are received as a line in serial port
        # so I think the rate should be (5 * gpsDataRate). Note that this is true in our UBlox-M8M module that we use
        # and you may have more or less than 5 formats. So in general the rate should be numOfGPSFormats * gpsDataRate.
        self._receiveLoopRate = rospy.Rate(5 * gpsDataRate)

        self._GPSPublisher = rospy.Publisher(gpsTopicName, NavSatFix, queue_size=1)
        self._GPSToRfConnectionPublisher = rospy.Publisher(rfConnectionTopicName, Mavlink, queue_size=1)
        self._GPSPublishBuffer = []
        self._publishLoopRate = rospy.Rate(gpsDataRate)
        self._GPSPublisherThread = threading.Thread(target=self._GPSPublisherThreadWorker)
        self._GPSPublisherThread.start()
        return

    def GPSDataReceiveLoop(self):
        while not rospy.is_shutdown():
            try:
                recvMsg = self._port.readline().decode('utf-8')
                if not recvMsg.startswith("$GNGGA"):
                    # I think, continue keyword will cause ignoring rate sleep at the end of while loop, so to have
                    # correct timing I think we should add rate sleep before continue statements. TODO: Test it!
                    self._receiveLoopRate.sleep()
                    continue
            except:
                self._receiveLoopRate.sleep()
                continue

            try:
                gpsPosition = self._parseGNGGA(recvMsg)
                self._GPSPublishBuffer.append(gpsPosition)
            except:
                print("Data is corrupted!")
                self._receiveLoopRate.sleep()
                continue

            self._receiveLoopRate.sleep()
        return

    def _initializePort(self, portAddress, baudRate):
        self._port = serial.Serial(portAddress, baudRate, timeout=1.0)
        if not self._port.is_open:
            self._port.open()
        return

    def _configUbloxModule(self):
        # AT COMMAND to set the gps rate to 5Hz: 0xC8 0x00 -> 200[ms] I think!
        ubx_com = b'\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A\xB5\x62\x06\x08\x00\x00\x0E\x30'
        # Sending command to ublox module to change the measurement rate. This change is not permanent and Restarting
        # the module resets the configs.
        self._port.write(ubx_com)
        return

    def _parseGNGGA(self, gpsMsgLine):
        '''
        converts the GPGGA/GNGGA data format to the degrees
        :param gpsMsgLine: A line of GPS in serial port
        :return: latitude, longitude and altitude in degrees
        '''
        gpsMsgFields = gpsMsgLine.split(",")
        latField = gpsMsgFields[2]
        lonField = gpsMsgFields[4]
        altField = gpsMsgFields[9]  # Height relative to MSL or geoid
        # Convert latField to lat in degrees DDMM.MMMMM
        lat = float(latField[:2]) + (float(latField[2:]) / 60.0)
        # Convert lonField to lon in degrees DDDMM.MMMMM
        lon = float(lonField[:3]) + (float(lonField[3:]) / 60.0)
        alt = float(altField)
        return [lat, lon, alt]

    def _GPSPublisherThreadWorker(self):
        while not rospy.is_shutdown():
            if len(self._GPSPublishBuffer):
                tgGPSPosition = self._GPSPublishBuffer.pop(0)
                self._publishToGCSRosNetwork(tgGPSPosition)
                self._publishToRfConnection(tgGPSPosition)
            else:
                rospy.loginfo("Not received any GPS data yet!")
            self._publishLoopRate.sleep()
        return

    def _publishToGCSRosNetwork(self, tgGPSPosition):
        # TODO: Construct the header in future if it is needed.
        msg = NavSatFix()
        msg.latitude, msg.longitude, msg.altitude = tgGPSPosition
        self._GPSPublisher.publish(msg)
        return

    def _publishToRfConnection(self, tgGPSPosition):
        msg = self._constructMavlinkMsg(tgGPSPosition)
        self._GPSToRfConnectionPublisher.publish(msg)
        return

    def _constructMavlinkMsg(self, tgGPSPosition):
        # For now, we only set position fields and other fields are set to zero.
        # TODO: In future if you need, you should set first field that is timestamp and also other data fields like
        #   velocity fields if they are needed.
        mavMsg = mavutil.mavlink.MAVLink_position_target_global_int_message(0, mavutil.mavlink.MAV_FRAME_GLOBAL_INT, 0,
                                                                            int(tgGPSPosition[0] * 1e7),
                                                                            int(tgGPSPosition[1] * 1e7),
                                                                            tgGPSPosition[2], 0, 0, 0, 0, 0, 0, 0, 0)
        mavMsg.pack(self._mavProtocolObj)
        return mavlink.convert_to_rosmsg(mavMsg)


if __name__ == "__main__":
    rospy.init_node("targetGPSReceiver", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB1")
    parser.add_argument("-b", "--baudrate", default=115200)
    parser.add_argument("-t", "--gpsTopicName", default="/target/globalPosition")
    parser.add_argument("-r", "--rfConnectionTopicName", default="/GCS/from")
    args = parser.parse_args()

    gpsDataReceiver = GPSDataReceiver(args.serial_port, args.baudrate, args.gpsTopicName, args.rfConnectionTopicName, 5)
    # Note: Receiver loop blocks the main thread execution.
    gpsDataReceiver.GPSDataReceiveLoop()
