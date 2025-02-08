import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface


class RosImageTopicCameraFrameCapture(CameraFrameCaptureInterface):
    def __init__(self, frame_source, frame_size=(1920, 1080), image_buffer_size=2):
        super().__init__(frame_source=frame_source, frame_size=frame_size, image_buffer_size=image_buffer_size)
        self._running = True
        self._cv_bridge = CvBridge()
        self._frame_sub = rospy.Subscriber(frame_source, Image, self._capturing, queue_size=1)
        return

    def _capturing(self, msg: Image):
        frame = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, self._frame_size)
        # Process the frame and add it to the image buffer
        if self._frame_processor:
            processed_frame = self._frame_processor.process_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        else:
            processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._addImageToBuffer(processed_frame)
        return

    def stop(self):
        rospy.loginfo("Stopping the video capture subscriber...")
        self._running = False
        self._frame_sub.unregister()
        rospy.loginfo("Frame capture subscriber unregistered.")
