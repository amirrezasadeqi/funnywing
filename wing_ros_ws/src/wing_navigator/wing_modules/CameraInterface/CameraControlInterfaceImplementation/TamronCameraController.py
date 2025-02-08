import csv

import rospy
from serial import Serial

from wing_modules.CameraInterface.CameraControlInterface import CameraControlInterface


class TamronCameraController(CameraControlInterface):
    def __init__(self, zoom_range, port, baudrate, preset_table_file):
        """
        @param preset_table_file: Absolute path to the preset table CSV file.
        """
        super().__init__(zoom_range)
        self._camera_connection = Serial(port, baudrate)
        rospy.loginfo(f"Tamron camera is connected to {port} with baudrate {baudrate}!")
        self._load_preset_table(preset_table_file)
        return

    def set_zoom(self, zoom):
        """
        @param zoom: zoom in hexadecimal number, from 0x0000 to 0x4000(more zoom).
        """
        p, q, r, s = self._get_hex_digits(zoom)
        zoom_cmd = bytes.fromhex(f"81 01 04 47 0{p} 0{q} 0{r} 0{s} FF")
        try:
            self._camera_connection.write(zoom_cmd)
            return True
        except Exception as e:
            rospy.logwarn(f"Failed to set zoom: {e}")
            return False

    def set_focus(self, focus):
        """
        @param focus: focus in hexadecimal number, from 0x1000(Far) to 0xB000(Near).
        """
        p, q, r, s = self._get_hex_digits(focus)
        focus_cmd = bytes.fromhex(f"81 01 04 48 0{p} 0{q} 0{r} 0{s} FF")
        self._camera_connection.write(focus_cmd)

    def set_preset_at_idx(self, preset_idx, do_mapping=True):
        if do_mapping:
            # mapping from 0-100 to the preset index in the preset table.
            step_size = 100 / (len(self._preset_table) - 1)
            preset_idx = int(preset_idx / step_size)
        p, q, r, s = self._get_hex_digits(self._preset_table[preset_idx]["zoom"])
        t, u, v, w = self._get_hex_digits(self._preset_table[preset_idx]["focus"])
        preset_cmd = bytes.fromhex(f"81 01 04 47 0{p} 0{q} 0{r} 0{s} 0{t} 0{u} 0{v} 0{w} FF")
        try:
            self._camera_connection.write(preset_cmd)
            return True
        except Exception as e:
            rospy.logwarn(f"Failed to set preset at index {preset_idx}: {e}")
            return False

    def _get_hex_digits(self, hex_number):
        # Convert the hexadecimal number to a string without the '0x' prefix and
        # 4 characters long, since we need exactly 4 characters for creating the
        # visca commands, otherwise there will be an error for not enough digits
        # to feed in the commands.
        hex_str = f"{hex_number:04x}".upper()
        # Return each character
        return [digit for digit in hex_str]

    def _load_preset_table(self, preset_table_file):
        with open(preset_table_file, mode='r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip the hint line
            next(reader)  # Skip the column name line
            for row in reader:
                self._preset_table.append({"preset_name": row[0], "zoom": int(row[1], 16), "focus": int(row[2], 16)})
        return

    def set_focus_mode(self, focus_mode):
        # TODO: Implement this method
        rospy.logwarn("This method is not implemented for Tamron camera controller yet.")
        return
