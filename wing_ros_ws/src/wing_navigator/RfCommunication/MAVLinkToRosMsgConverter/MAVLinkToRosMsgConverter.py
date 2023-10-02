from typing import Dict
from RfCommunication.MAVLinkToRosMsgConverter import ConverterClasses
from RfCommunication.RfConnection.ConnectionInterface.ConnectionInterface import ConnectionInterface
from RfCommunication.MAVLinkToRosMsgConverter.ConverterClasses import all_message_types_to_mavros_msgs_Mavlink


class MAVLinkToRosMsgConverter(object):
    def __init__(self, message, rfConnection: ConnectionInterface):
        """
        A class to manage the conversion of mavlink messages to ROS messages.

        Conversion of mavlink message needs mav field of the mavlink connection(instance of MAVLink class)
        so to be perfect we pass connection here to use it in the conversions, otherwise you can simply create
        an instance of MAVLink and use for conversion.

        @param message: MAVLink_<message_type> message.
        """
        self._message = message
        self._rfConnection = rfConnection
        self._publisherConfig = None
        self._converter = None
        return

    def setMessage(self, message):
        self._message = message
        return

    def setupConverter(self, publisherConfig: Dict):
        """

        @param publisherConfig: Dictionary containing the publisher information which is necessary to
        choose the mavlink to ROS message converter.
        """
        self._publisherConfig = publisherConfig
        self._converter = self._getConverter()
        return

    def convert(self):
        """

        @return: A ROS message containing mavlink message data
        """
        return self._converter.convertToRosMsg()

    def _getConverter(self):
        try:
            converterClassName = self._getConverterClassName()
            if "all_message_types_to_mavros_msgs_Mavlink" == converterClassName:
                return all_message_types_to_mavros_msgs_Mavlink(self._message, self._rfConnection)
            else:
                return getattr(ConverterClasses, converterClassName)(self._message)
        except Exception:
            raise Exception("Conversion is not available at the moment, Please Implement it!")

    def _getConverterClassName(self):
        """
        Using the name of the mavlink message(e.g. "HEARTBEAT") and its fields(e.g. "custom_mode" in "HEARTBEAT"
         message) and ROS message package and type(e.g. std_msgs package and Float64 message type) creates a name
         for the converter class that we should implement to do the conversion.
         Converter class name format is <lowercase mavlink message name>_[list of fields separated by _]_to_<ROS message
         package>_<ROS message type>
         example class name: global_position_int_lat_lon_alt_to_sensor_msgs_NavSatFix

        Regardless of mavlink message type, we use mavros_msgs/Mavlink publisher and the procedure of this conversion
        is the same for all mavlink messages, so we return the same converter class
        ("all_message_types_to_mavros_msgs_Mavlink")for all of them.
        """
        pubConfig = self._publisherConfig
        if "ALL_MESSAGE_TYPES" == pubConfig["onMessage"]:
            return "all_message_types_to_mavros_msgs_Mavlink"
        msgPack = pubConfig["msgPack"]
        msgType = pubConfig["msgType"]
        converterClassName = self._message.get_type().lower()
        if "messageFields" in pubConfig:
            for field in pubConfig["messageFields"]:
                converterClassName += f"_{field}"
        converterClassName += "_to_" + msgPack + "_" + msgType
        return converterClassName
