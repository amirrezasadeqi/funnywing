import threading
import rospy
from mavros import mavlink
from mavros_msgs.msg import Mavlink

from RfCommunication.RfConnection.RfConnection import RfConnection
from RfCommunication.Job.Factory.JobFactory import JobFactory
from RfCommunication.Job.Interface.JobInterface import JobInterface


class RfCommunicationHandler(object):
    def __init__(self, rfConnection: RfConnection, jobFactory: JobFactory, systemName: str,
                 inBufWaitForMsg: float = 1e-4):
        """
        RfCommunicationHandler constructor. This class assembles all parts of the architecture
        planned to handle the RF communication in funnywing project.

        _transRosMsgThread thread can be replaced by a ROS subscriber and its callback.
        but in that way we need rospy.spin() to be called outside which is
        not intuitive (since someone may forget to call it outside) or inside
        which make this constructor blocking. So we use a separated thread containing
        rospy.spin() and solves these problems.

        @type rfConnection: RfConnection
        """
        self._rfConnection = rfConnection
        self._jobFactory = jobFactory
        self._inBufWaitForMsg = inBufWaitForMsg
        self._fromRosTopic = "/from_" + systemName + "_ros"
        self._recvMavMsgThread = None
        self._transRosMsgThread = None
        self._mavrosMsgSubscriber = None

        self._handleReceivedMavlinkMsg()
        self._handleTransferringRosMsg()
        return

    def _handleReceivedMavlinkMsg(self):
        self._recvMavMsgThread = threading.Thread(target=self._recvMavMsgCallback)
        self._recvMavMsgThread.start()
        return

    def _recvMavMsgCallback(self):
        while not rospy.is_shutdown():
            inMavMsg = self._rfConnection.read()
            if inMavMsg:
                self._jobFactory.setMessage(inMavMsg)
                job: JobInterface = self._jobFactory.createJob()
                job.runJob()
            else:
                # sleep to prevent from wasting performance when there is no message
                rospy.sleep(self._inBufWaitForMsg)
        return

    def _handleTransferringRosMsg(self):
        self._transRosMsgThread = threading.Thread(target=self._transRosMsgCallback)
        self._transRosMsgThread.start()
        return

    def _transRosMsgCallback(self):
        # TODO: Consider using mavros MAVLINK message type for subscribing/publishing mavlink
        #   messages and commands in ROS network.
        self._mavrosMsgSubscriber = rospy.Subscriber(self._fromRosTopic, Mavlink, self._mavrosMsgSubscriberCallback)
        rospy.spin()
        return

    def _mavrosMsgSubscriberCallback(self, mavrosMsg: Mavlink):
        """
        @param mavrosMsg: Mavlink
        @return:
        """
        # convert Mavlink message to mavutil.mavlink.MAVLink_<message_type>
        mavlinkMsg = self._mavrosToMavlink(mavrosMsg)
        # write message to the connection
        self._rfConnection.write(mavlinkMsg)
        return

    def _mavrosToMavlink(self, mavrosMsg: Mavlink):
        """
        To Understand why we convert mavrosMsg to bytearray you should read
        your desired dialect module(e.g. funnywing.py). It is hard to track
        what happens, but it is manageable.
        @param mavrosMsg: Mavlink
        @return: mavutil.mavlink.MAVLink_<message_type>. Note that mavutil.mavlink is
        actually a reference to the selected dialect module we use in project. For example
        here it is a reference to pymavlink.dialects.v20.funnywing module, so we can use
        this for getting MAVLink_<message_type> and the other things.
        """
        bits = mavlink.convert_to_bytes(mavrosMsg)
        mavlinkMsg = self._rfConnection.getPort().mav.decode(bits)
        return mavlinkMsg
