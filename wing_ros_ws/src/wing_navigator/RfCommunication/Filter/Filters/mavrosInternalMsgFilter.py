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

        UNKNOWN_11020 message is a MAVLink_message derived from MAVLink_unknown for messages not defined in dialect XML.
        At least in simulation, sometimes we have them. So, it also is filtered here.

        @param configPath:
        """
        self._config = self._readFilterConfig(configPath)
        return

    def msgIsPassed(self, message) -> bool:
        """
        Message ID of the UNKNOWN_11020 is 11020. This message at least in simulation is published periodically, so it
        is filtered here.

        @param message: mavros_msgs/Mavlink
        @return: boolean, True if the message is going to be written on the RF connection.
        """
        return (11020 != message.msgid) and (self._config["sourceSystem"] == message.sysid)

    def _readFilterConfig(self, configPath):
        configFile = open(configPath)
        return json.load(configFile)["mavrosInternalMsgFilter"]
