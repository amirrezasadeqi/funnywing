import json

from RfCommunication.Filter.Interface.FilterInterface import FilterInterface


class mavrosInternalMsgFilter(FilterInterface):
    def __init__(self, configPath):
        """

        This Filter returns True only if the Mavlink rosMsg source is the system we specify in the sourceSystem field
        of the configurations, This way, we can filter the messages coming from the internal things of the mavros and
        the mavros heartbeat( These messages previously caused some bugs in seeing the correct flight mode in the GCS
        side.). Also, in this way the message coming from other places(probably unknown or unwanted places) can be
        dropped or filtered.

        @param configPath:
        """
        self._config = self._readFilterConfig(configPath)
        return

    def msgIsPassed(self, message) -> bool:
        """

        @param message: mavros_msgs/Mavlink
        @return: boolean, True if the message is going to be written on the RF connection.
        """
        return self._config["sourceSystem"] == message.sysid

    def _readFilterConfig(self, configPath):
        configFile = open(configPath)
        return json.load(configFile)["mavrosInternalMsgFilter"]
