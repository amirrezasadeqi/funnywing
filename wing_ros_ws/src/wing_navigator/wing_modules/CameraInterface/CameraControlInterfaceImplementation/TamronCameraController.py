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
        Caution: Use set_focus_mode to set focus mode to manual before calling this method.
        @param focus: focus in hexadecimal number, from 0x1000(Far) to 0xB000(Near).
        """
        p, q, r, s = self._get_hex_digits(focus)
        focus_cmd = bytes.fromhex(f"81 01 04 48 0{p} 0{q} 0{r} 0{s} FF")
        self._camera_connection.write(focus_cmd)

    def set_preset_at_idx(self, preset_idx, do_mapping=True):
        """
        Caution: Calling this method will change camera focus mode to the manual mode! So set focus mode to auto when
        you need autofocus mode somewhere else.
        """
        # Set focus mode to manual to be able to set presets, since a preset is a combination of zoom and focus.
        self.set_focus_mode("manual")
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
        """
        @param focus_mode: "manual" or "auto"
        @return: True if the focus mode command was written successfully, False otherwise.
        """
        focus_mode_map = {
            "auto": '2',
            "manual": '3'
        }
        set_focus_mode_cmd = bytes.fromhex(f"81 01 04 38 0{focus_mode_map[focus_mode]} FF")
        try:
            self._camera_connection.write(set_focus_mode_cmd)
            return True
        except Exception as e:
            rospy.logwarn(f"Failed to set focus mode to {focus_mode}: {e}")
            return False

    def _get_int_from_zoom_query(self, zoom_query):
        val = ''
        for i, d in enumerate(zoom_query[2:-1].hex()):
            if i % 2:
                val += d
        return int(val.upper(), 16)

    def _read_zoom_query(self):
        """
        @return: returns a zoom query value as integer number. The value for tamron camera is 0x0000 to 0x4000.
        """
        # A buffer to store the last two bytes read from the camera connection for finding the header of the zoom query.
        buffer = b''
        while True:
            # ! Note !: The reading actions may need timeout, otherwise the guidance loop may be blocked. Expiration of
            # the timeout does not throw any exceptions or errors and just returns empty b'' and I think this can cause
            # infinite loop or blocking of the guidance loop. So in the future, if there would be some bugs like block
            # -ing the guidance loop, here can be the potential cause. So if there would be a bug for this, consider
            # handling it by for example detecting timeouts and returning None for zoom_query. So do that in the future
            # if you would need it!
            buffer += self._camera_connection.read(1)
            if len(buffer) > 2:
                buffer = buffer[-2:]
            # zoom query header is "90 50"
            if buffer == b'\x90\x50':
                # zoom query is 7 bytes long, starting with '90 50' and ending with 'FF'.
                zoom_query = buffer + self._camera_connection.read(5)
                return self._get_int_from_zoom_query(zoom_query)

    def get_zoom(self):
        try:
            get_zoom_cmd = bytes.fromhex(f"81 09 04 47 FF")
            # Flush input buffer before sending command to get the last command response or actual the latest zoom
            # query. Note that this reset can cause problems if there are other commands by other threads or processes
            # and this reset may remove the response of those commands. So in the future, if there are other commands
            # that we need to read their responses, we need to handle this issue for example by using a queue of
            # commands which must be handled, and they would be handled by a separated dedicated thread and here we just
            # need the command to the command query.
            self._camera_connection.reset_input_buffer()
            self._camera_connection.write(get_zoom_cmd)
            zoom_query = self._read_zoom_query()
            # Convert zoom_query(0x0000-0x4000) to zoom_level number with assumption that Tamron is 1X-10X camera.
            zoom_level = 1.0 + (zoom_query / 16384) * (10.0 - 1.0)
            return zoom_level
        except Exception as e:
            rospy.logwarn(f"Failed to get current zoom level of Tamron camera: {e}")
            return None
