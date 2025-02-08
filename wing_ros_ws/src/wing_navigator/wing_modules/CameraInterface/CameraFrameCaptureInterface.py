from collections import deque

from PySide2.QtCore import QObject, Signal


class CameraFrameCaptureInterface(QObject):
    newFrameCaptured = Signal()

    def __init__(self, frame_source, frame_size, image_buffer_size):
        if not hasattr(self, 'stop'):
            raise NotImplementedError(f"{self.__class__.__name__} must implement method 'stop'")
        super().__init__()
        self._frame_source = frame_source
        # frame size: (width, height)
        self._frame_size = frame_size
        self._image_buffer = deque(maxlen=image_buffer_size)
        self._frame_processor = None
        return

    def get_frame(self):
        try:
            return self._image_buffer.popleft()
        except IndexError:
            print("Frame buffer is empty!")
            return None

    def stop(self):
        raise NotImplementedError(f"{self.__class__.__name__} must implement method 'stop'")

    def set_frame_processor(self, frame_processor):
        self._frame_processor = frame_processor
        return

    def _addImageToBuffer(self, processed_frame):
        self._image_buffer.append(processed_frame)
        self.newFrameCaptured.emit()
        return
