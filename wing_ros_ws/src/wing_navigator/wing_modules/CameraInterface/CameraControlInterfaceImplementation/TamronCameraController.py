from wing_modules.CameraInterface.CameraControlInterface import CameraControlInterface


class TamronCameraController(CameraControlInterface):
    def __init__(self, zoom_range, port, baudrate):
        super().__init__(zoom_range)
        print(f"Tamron Camera Controller connecting at port: {port}, baudrate: {baudrate}")
        return

    def set_zoom(self, zoom):
        print("The method is not implemented yet.")
        return True

    def set_focus(self, focus):
        print("The method is not implemented yet.")

    def set_preset_at_idx(self, preset_idx, do_mapping=True):
        print("The method is not implemented yet.")
        return True
