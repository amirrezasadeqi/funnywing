from abc import ABC, abstractmethod

from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface
from RfCommunication.MAVLinkToRosMsgConverter.MAVLinkToRosMsgConverter import MAVLinkToRosMsgConverter


class JobInterface(ABC):

    def __init__(self, message, rfConnection: ConnectionInterface, mavrosPublishers, system, component):
        """
        Base Interface class for Using Job classes inside other codes
        self._srcSystem and self._srcComponent are address of message sender

        @param message: This is a mavlink message. Not the base class
         MAVLink_message but its child classes(e.g. MAVLink_heartbeat_message)
         . since we need to pass it to job class and use the attributes of those
         child classes in there.
        @param rfConnection: ConnectionInterface, Adding this parameter to Jobs to create for them
        the opportunity to do the things which are more related to a mavlink connection and not for
        some specific application. Use this only if necessary and try to use the connections in user
        program and client side to prevent the data transmissions to be distributed all over the codes.
        @param mavrosPublishers: List of dictionaries containing ROS publishers and their corresponding configs coming
        from JobFactory using MavrosPublishManager
        @param system: Identifier for the system that we are on, and we create the job for. source and component
        will be probably identified by Mavlink system and component type enums.
        @param component: Identifier for the component that we are on, and we create job for
        """
        self._message = message
        self._rfConnection = rfConnection
        self._mavrosPublishers = mavrosPublishers
        self._system = system
        self._component = component
        self._srcSystem = self._message.get_header().srcSystem
        self._srcComponent = self._message.get_header().srcComponent

        self._mavlinkToRosMsgConverter = MAVLinkToRosMsgConverter(self._message, self._rfConnection)
        return

    def runJob(self):
        self._publishMessageToRos()
        self._doJob()
        return

    def setMessage(self, message):
        self._message = message
        return

    def getMessage(self):
        return self._message

    def getRfConnection(self) -> ConnectionInterface:
        return self._rfConnection

    def _publishMessageToRos(self):
        for publisher in self._mavrosPublishers:
            self._mavlinkToRosMsgConverter.setPublisherConfig(publisher["publisherConfig"])
            msg = self._mavlinkToRosMsgConverter.convert()
            publisher["publisherObject"].publish(msg)
        return

    @abstractmethod
    def _doJob(self):
        pass
