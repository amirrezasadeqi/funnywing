from abc import ABC, abstractmethod


class CameraControlInterface(ABC):
    def __init__(self, zoom_range):
        """
        preset table is a list of preset dictionaries with below structure:
        {"preset_name": name[string], "zoom": zoom_value[HEX], "focus": focus_value[HEX]}
        """
        # zoom range is for example a tuple (1, 100) which means that the zoom level can be set from 1 to 100.
        self._zoom_range = zoom_range
        self._preset_table = []
        return

    @abstractmethod
    def set_zoom(self, zoom):
        """
        This methode implements changing of the camera zoom level.
        @return: True if the zoom was set successfully, False otherwise.
        """
        pass

    @abstractmethod
    def set_focus(self, focus):
        """
        This methode implements changing of the camera focus level.
        """
        pass

    @abstractmethod
    def set_preset_at_idx(self, preset_idx, do_mapping=True):
        """
        This methode implements changing of the camera preset to the preset with id == preset_idx.

        If the do_mapping is True, the preset_idx value would be in 0-100 range and this range should be mapped to the
        real range of indices in the preset table, otherwise the raw value of preset_idx should be used.
        @return: True if the preset was set successfully, False otherwise.
        """
        pass

    def set_preset_at_name(self, preset_name):
        """
        This methode implements changing of the camera preset to the preset with preset_name == preset_name.
        """
        pass

    def define_preset(self, preset):
        """
        This methode implements defining of a new camera preset level.
        """
        pass

    def load_preset_table(self, preset_table_file):
        """
        This methode implements loading of the camera preset table from a file.
        """
        pass

    def set_zoom_mode(self, zoom_mode):
        """
        This methode implements changing of the camera zoom mode. The camera zoom mode can be for example continuous or
        discrete.
        """
        pass

    def set_focus_mode(self, focus_mode):
        """
        This methode implements changing of the camera focus mode. The camera focus mode can be for example auto or
        manual.
        """
        pass

    def set_exposure_mode(self, exposure_mode):
        """
        This methode implements changing of the camera exposure mode. The camera exposure mode can be for example auto
        or manual.
        """
        pass

    def set_white_balance_mode(self, white_balance_mode):
        """
        This methode implements turning ON/OFF the camera white balance mode.
        """
        pass
