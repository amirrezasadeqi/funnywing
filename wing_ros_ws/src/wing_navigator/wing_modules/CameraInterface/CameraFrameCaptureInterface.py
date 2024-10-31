from collections import deque

from PySide2.QtCore import QObject, Signal


class CameraFrameCaptureInterface(QObject):
    newFrameCaptured = Signal()

    def __init__(self, frame_source, image_buffer_size=2):
        if not hasattr(self, 'stop'):
            raise NotImplementedError(f"{self.__class__.__name__} must implement method 'stop'")
        super().__init__()
        self._frame_source = frame_source
        self._image_buffer = deque(maxlen=image_buffer_size)
        return

    def get_frame(self):
        try:
            return self._image_buffer.popleft()
        except IndexError:
            print("Frame buffer is empty!")
            return None

    def stop(self):
        raise NotImplementedError(f"{self.__class__.__name__} must implement method 'stop'")

    def _addImageToBuffer(self, processed_frame):
        self._image_buffer.append(processed_frame)
        self.newFrameCaptured.emit()
        return
