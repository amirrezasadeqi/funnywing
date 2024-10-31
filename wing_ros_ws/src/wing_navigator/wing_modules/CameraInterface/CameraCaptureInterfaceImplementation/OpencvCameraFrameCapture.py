from threading import Thread

import cv2
import rospy

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface


class OpencvCameraFrameCapture(CameraFrameCaptureInterface):
    def __init__(self, frame_source, image_buffer_size=2):
        super().__init__(frame_source=frame_source, image_buffer_size=image_buffer_size)
        self._cap = None
        self._running = True
        self._frameCaptureThread = Thread(target=self._capturing)
        self._frameCaptureThread.start()
        return

    def _capturing(self):
        # VideoCapture constructed in the thread target to prevent blocking, which in turn blocks the event loop of the
        # Qt and the program won't start without video stream source.
        self._cap = cv2.VideoCapture(self._frame_source)
        while self._running:
            try:
                ret, frame = self._cap.read()
                if not ret:
                    rospy.loginfo("Nothing captured!")
                    continue
                # Process the frame and add it to the image buffer
                processed_frame = self._process_frame(frame)
                self._addImageToBuffer(processed_frame)
            except Exception as e:
                print(e)
                break
        return

    def _process_frame(self, frame):
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        processed_frame = cv2.rectangle(processed_frame, (50, 50), (200, 200), (0, 255, 0), 3)
        return processed_frame

    def stop(self):
        rospy.loginfo("Stopping the video capture thread...")
        self._running = False
        self._frameCaptureThread.join()
        self._cap.release()
        rospy.loginfo("Frame capture thread stopped.")
