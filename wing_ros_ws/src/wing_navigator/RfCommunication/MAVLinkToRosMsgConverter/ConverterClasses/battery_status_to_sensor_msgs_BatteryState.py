from std_msgs.msg import Header
from sensor_msgs.msg import BatteryState


class battery_status_to_sensor_msgs_BatteryState(object):
    def __init__(self, message):
        """

        @param message: MAVLink_<message type> mavlink message. Here the message is MAVLink_battery_status_message.
        """
        self._message = message
        return

    def convertToRosMsg(self):
        """
        Converts BATTERY_STATUS mavlink message to sensor_msgs/BatteryState message.
        """
        # TODO: Set other ROS message fields like header and so on.
        rosMsg = BatteryState()
        rosMsg.header = self._getRosMsgHeader()
        rosMsg.cell_voltage = [milliVolt / 1000.0 for milliVolt in self._message.voltages]  # Voltages are in mV
        rosMsg.voltage = sum(rosMsg.cell_voltage)
        rosMsg.percentage = self._message.battery_remaining
        rosMsg.temperature = self._message.temperature / 100.0  # Temperature is in centi degC
        return rosMsg

    def _getRosMsgHeader(self) -> Header:
        header = Header()
        return header
