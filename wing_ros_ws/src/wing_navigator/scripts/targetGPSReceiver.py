import argparse

import rospy
import serial
from sensor_msgs.msg import NavSatFix


class GPSDataReceiver(object):
    def __init__(self, portAddress, baudRate, gpsTopicName, gpsDataRate):
        self._port = None
        self._initializePort(portAddress, baudRate)
        self._configUbloxModule()
        self._GPSPublisher = rospy.Publisher(gpsTopicName, NavSatFix, queue_size=1)

        # TODO: check if you should use (measurement rate) * (number of gps data line) for the rate or not!
        # Each GPS data advertised in 5 different GPS formats and each of formats are received as a line in serial port
        # so I think the rate should be (5 * gpsDataRate).
        self._loopRate = rospy.Rate(5 * gpsDataRate)
        return

    def GPSDataPublishLoop(self):
        while not rospy.is_shutdown():
            # TODO: Construct the header in future if it is needed.
            msg = NavSatFix()

            try:
                recvMsg = self._port.readline().decode('utf-8')
                if not recvMsg.startswith("$GNGGA"):
                    # I think, continue keyword will cause ignoring rate sleep at the end of while loop, so to have
                    # correct timing I think we should add rate sleep before continue statements. TODO: Test it!
                    self._loopRate.sleep()
                    continue
            except:
                self._loopRate.sleep()
                continue

            try:
                msg.latitude, msg.longitude, msg.altitude = self._parseGNGGA(recvMsg)
                self._GPSPublisher.publish(msg)
            except:
                print("Data is corrupted!")
                self._loopRate.sleep()
                continue

            self._loopRate.sleep()
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
        return lat, lon, alt


if __name__ == "__main__":
    rospy.init_node("/targetGPSReceiver", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--serial_port", default="/dev/ttyUSB1")
    parser.add_argument("-b", "--baudrate", default=9600)
    args = parser.parse_args()

    gpsDataReceiver = GPSDataReceiver(args.serial_port, args.baudrate, "/target/globalPosition", 5)
    gpsDataReceiver.GPSDataPublishLoop()
