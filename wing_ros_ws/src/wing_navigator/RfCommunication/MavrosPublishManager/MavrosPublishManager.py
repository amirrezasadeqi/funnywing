import json
import rospy
import threading
from importlib import import_module
from typing import List, Dict

from RfCommunication.RfConnection.RfConnection import RfConnection
from RfCommunication.MAVLinkToRosMsgConverter.MAVLinkToRosMsgConverter import MAVLinkToRosMsgConverter


class MavrosPublishManager(object):
    def __init__(self, configPath, rfConnection: RfConnection, publishBufferWaitForMsg=1e-4):
        self._configs = self._readConfigs(configPath)
        self._rfConnection = rfConnection
        self._publishBufferWaitForMsg = publishBufferWaitForMsg
        self._publishers = {}
        self._publishBuffer = []
        self._mavlinkToRosMsgConverter = None
        self._publisherThread = None
        self._initializePublishers()
        self._publishMsgsToRos()
        return

    def addToPublishBuffer(self, mavMsg):
        """

        @param mavMsg: MAVLink_<message_type> to be published.
        """
        self._publishBuffer.append(mavMsg)
        return

    def _getPublishers(self, msgType: str):
        """
        Gives a list of dictionaries that each of them containing publisher object and the corresponding
        configs for that object(for considering the corner cases, for example GLOBAL_POSITION_INT can be
        converted to std_msgs/Float64 to get different fields in ROS, like heading, relative altitude and
        so on.)
        @return: List of dictionaries containing ROS publishers and their corresponding configs.
        """
        publishers = []
        if "UNSUPPORTED_MESSAGE" == msgType:
            return publishers
        # For the supported messages
        for config in self._configs:
            if self._isConfigForMsgType(config, msgType):
                publisher = {"publisherObject": self._publishers[config["name"]], "publisherConfig": config}
                publishers.append(publisher)

        return publishers

    def _readFromPublisherBuffer(self):
        if len(self._publishBuffer):
            return self._publishBuffer.pop(0)
        else:
            return None

    def _isConfigForMsgType(self, config, msgType):
        return (config["onMessage"] in [msgType, "ALL_MESSAGE_TYPES"]) and config["enable"]

    def _readConfigs(self, configPath) -> List[Dict]:
        configFile = open(configPath)
        return json.load(configFile)

    def _initializePublishers(self):
        for config in self._configs:
            if config["enable"]:
                pubName = config["name"]
                topicName = config["topicName"]
                msgType = self._getPublisherMsgType(config["msgPack"], config["msgType"])
                queueSize = config["queueSize"]
                self._publishers[pubName] = rospy.Publisher(topicName, msgType, queue_size=queueSize)
        return

    def _getPublisherMsgType(self, msgPack, msgType):
        return getattr(import_module(msgPack + ".msg"), msgType)

    def _publishMsgsToRos(self):
        self._publisherThread = threading.Thread(target=self._publisherThreadCallback)
        self._publisherThread.start()
        return

    def _publisherThreadCallback(self):
        while not rospy.is_shutdown():
            mavMsg = self._readFromPublisherBuffer()
            if mavMsg:
                self._publishMavMsg(mavMsg)
            else:
                # sleep to prevent from wasting performance when there is no message to publish
                rospy.sleep(self._publishBufferWaitForMsg)
        return

    def _publishMavMsg(self, mavMsg):
        """

        @param mavMsg: MAVLink_<message_type>.
        """
        mavrosPublishers = self._getPublishers(mavMsg.get_type())
        self._mavlinkToRosMsgConverter = MAVLinkToRosMsgConverter(mavMsg, self._rfConnection)
        for publisher in mavrosPublishers:
            try:
                self._mavlinkToRosMsgConverter.setupConverter(publisher["publisherConfig"])
            except Exception as e:
                rospy.logwarn(e)
                continue
            msg = self._mavlinkToRosMsgConverter.convert()
            publisher["publisherObject"].publish(msg)
        return
