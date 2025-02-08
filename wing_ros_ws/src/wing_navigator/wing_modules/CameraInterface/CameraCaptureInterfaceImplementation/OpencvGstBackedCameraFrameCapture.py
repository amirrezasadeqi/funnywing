from threading import Thread

import cv2
import rospy

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface


class OpencvGstBackedCameraFrameCapture(CameraFrameCaptureInterface):
    def __init__(self, frame_source, frame_size=(1920, 1080), image_buffer_size=2, use_nvidia=False):
        super().__init__(frame_source=frame_source, frame_size=frame_size, image_buffer_size=image_buffer_size)
        self._cap = None
        self._running = True
        self._use_nvidia = use_nvidia
        self._gstPipeline = (
            'rtspsrc location={} latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false').format(
            frame_source)
        # For using with nvidia gpu if supported
        # self._gstPipeline = ('rtspsrc location={} latency=0 ! '
        #                      'rtph264depay ! h264parse ! nvv4l2decoder ! '
        #                      'nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! '
        #                      'appsink sync=false ').format(frame_source)
        self._frameCaptureThread = Thread(target=self._capturing)
        self._frameCaptureThread.start()
        return

    def _capturing(self):
        # VideoCapture constructed in the thread target to prevent blocking, which in turn blocks the event loop of the
        # Qt and the program won't start without video stream source.
        self._cap = cv2.VideoCapture(self._gstPipeline, cv2.CAP_GSTREAMER)
        while self._running:
            try:
                ret, cap_frame = self._cap.read()
                if not ret:
                    rospy.loginfo("Nothing captured!")
                    continue
                if self._use_nvidia:
                    # For nvidia decoder, this conversion is needed otherwise, the frame is something gray like.
                    frame = cv2.cvtColor(cap_frame, cv2.COLOR_BGRA2BGR)
                else:
                    frame = cap_frame
                # resize the image to proper size. For example in runcam6, the image size of the stream is 4K(I think)
                # and is not changeable, so an area of the image is displayed on the GUI(since it is big) and it is
                # horizontally stripped. This resizing solved the problem.
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
