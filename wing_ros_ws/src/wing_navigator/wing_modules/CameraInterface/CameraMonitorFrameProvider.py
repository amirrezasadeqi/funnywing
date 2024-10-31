from PySide2.QtCore import QObject, Signal, Slot
from PySide2.QtGui import QImage, QColor
from PySide2.QtQuick import QQuickImageProvider

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface


class CameraMonitorFrameProvider(QObject, QQuickImageProvider):
    cameraMonitorFrameProviderQuited = Signal()

    def __init__(self, frame_capture: CameraFrameCaptureInterface, backFrontConnection, frame_size=(1080, 1920, 3)):
        super().__init__()
        QQuickImageProvider.__init__(self, QQuickImageProvider.Image)
        self._frame_size = frame_size
        self._frame_capture = frame_capture
        self._backFrontConnection = backFrontConnection
        self._frame_capture.newFrameCaptured.connect(self.onNewFrameCaptured)
        self._backFrontConnection.closeBackendSignal.connect(self.stop)
        return

    def requestImage(self, id, size, requestedSize):
        frame = self._frame_capture.get_frame()
        if frame is not None:
            height, width, channel = self._frame_size
            bytes_per_line = channel * width
            Qframe = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
        else:
            Qframe = QImage(self._frame_size[1], self._frame_size[0], QImage.Format_RGB888)
            Qframe.fill(QColor("black"))
        return Qframe

    @Slot()
    def stop(self):
        print("Stopping frame provider...")
        self._frame_capture.stop()
        self.cameraMonitorFrameProviderQuited.emit()
        print("Frame provider stopped.")

    @Slot()
    def onNewFrameCaptured(self):
        self._backFrontConnection.updateCameraMonitorFrame.emit()
        return
