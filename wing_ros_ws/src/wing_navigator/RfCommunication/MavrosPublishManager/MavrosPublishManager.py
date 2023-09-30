import json
import rospy
from importlib import import_module
from typing import List, Dict


class MavrosPublishManager(object):
    def __init__(self, configPath):
        self._configs = self._readConfigs(configPath)
        self._publishers = {}
        self._initializePublishers()
        return

    def getPublishers(self, msgType: str):
        """
        Gives a list of dictionaries that each of them containing publisher object and the corresponding
        configs for that object(for considering the corner cases, for example GLOBAL_POSITION_INT can be
        converted to std_msgs/Float64 to get different fields in ROS, like heading, relative altitude and
        so on.)
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
