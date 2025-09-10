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
        if use_nvidia:  # used on jetson
            self._gstPipeline = ("rtspsrc location={rtsp_url} latency=0 drop-on-latency=true ! rtph264depay ! "
                                 "video/x-h264,width={width},height={height},framerate=30/1 ! queue ! "
                                 "nvv4l2decoder enable-max-performance=1 disable-dpb=true ! queue ! "
                                 "video/x-raw(memory:NVMM),width={width},height={height},framerate=30/1,format=(string)NV12 ! "
                                 "nvvidconv ! video/x-raw,width={width},height={height},framerate=30/1,format=(string)BGRx ! "
                                 "videoconvert primaries-mode=fast ! "
                                 "video/x-raw,width={width},height={height},framerate=30/1,format=(string)BGR ! "
                                 "appsink sync=false drop=true"
                                 ).format(rtsp_url=frame_source, width=frame_size[0], height=frame_size[1])
        else:  # used on GCS
            self._gstPipeline = ('rtspsrc location={} latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! '
                                 'videoconvert ! appsink sync=false').format(frame_source)
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
                # resize the image to proper size. For example in runcam6, the image size of the stream is 4K(I think)
                # and is not changeable, so an area of the image is displayed on the GUI(since it is big) and it is
                # horizontally stripped. This resizing solved the problem.
                frame = cv2.resize(cap_frame, self._frame_size)
                # Process the frame and add it to the image buffer
                if self._frame_processor:
                    processed_frame = self._frame_processor.process_frame(frame)
                else:
                    processed_frame = frame
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
