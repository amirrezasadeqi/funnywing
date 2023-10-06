import rospy
import threading
from mavros import mavlink
from mavros_msgs.msg import Mavlink

from RfCommunication.Job.Factory.JobFactory import JobFactory
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface
from RfCommunication.MavrosPublishManager.MavrosPublishManager import MavrosPublishManager


class RfCommunicationHandler(object):
    def __init__(self, rfConnection: ConnectionInterface, jobFactory: JobFactory,
                 mavrosPublisherManager: MavrosPublishManager, systemID, componentID, fromRosTopic,
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
        @param fromRosTopic: This is the name of the mavros_msgs/Mavlink topic that this node subscribes to and converts
        its data to mavlink message and sends through mavlink port. This can be used for forwarding the mavlink data
        coming from FCU by mavros node on RPI. Also, you can use it in GCS for sending commands to RPI by just creating
        MAVLink_<message_type> object and convert it to mavros_msgs/Mavlink and publishing it to this topic.
        """
        self._rfConnection = rfConnection
        self._jobFactory = jobFactory
        self._mavrosPublisherManager = mavrosPublisherManager
        self._inBufWaitForMsg = inBufWaitForMsg
        self._fromRosTopic = fromRosTopic
        self._systemID = systemID
        self._componentID = componentID
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
                self._mavrosPublisherManager.addToPublishBuffer(inMavMsg)
                job = self._jobFactory.createJob(inMavMsg)
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
        if self._filterMavrosInternalMessages(mavlinkMsg):
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

    def _filterMavrosInternalMessages(self, mavlinkMsg):
        """
        This function returns True only if the mavlink message source is the system we specify in the self._systemID,
        This way, we can filter the messages coming from the internal things of the mavros and the mavros heartbeat(
        These messages previously caused some bugs in seeing the correct flight mode in the GCS side.). Also, in this
        way the message coming from other place(probably unknown or unwanted palces) can be dropped or filtered.
        @param mavlinkMsg: MAVLink_<message_type>
        @return: boolean, True if the message is going to be written on the RF connection.
        """
        return self._systemID == mavlinkMsg.get_header().srcSystem
