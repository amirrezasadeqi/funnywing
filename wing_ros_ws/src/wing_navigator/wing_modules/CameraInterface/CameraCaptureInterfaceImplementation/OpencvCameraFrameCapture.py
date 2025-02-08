from threading import Thread

import cv2
import rospy

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface


class OpencvCameraFrameCapture(CameraFrameCaptureInterface):
    def __init__(self, frame_source, frame_size=(1920, 1080), image_buffer_size=2):
        super().__init__(frame_source=frame_source, frame_size=frame_size, image_buffer_size=image_buffer_size)
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
                frame = cv2.resize(frame, self._frame_size)
                # Process the frame and add it to the image buffer
                if self._frame_processor:
                    processed_frame = self._frame_processor.process_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                else:
                    processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self._addImageToBuffer(processed_frame)
            except Exception as e:
                print(e)
                break
        return

    def stop(self):
        rospy.loginfo("Stopping the video capture thread...")
        self._running = False
        self._frameCaptureThread.join()
        self._cap.release()
        rospy.loginfo("Frame capture thread stopped.")
