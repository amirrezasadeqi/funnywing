from threading import Thread

import cv2
import gi
import numpy as np
import rospy

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
# GstApp is not used but should be imported, otherwise you get below error/warning and nothing is captured:
# GstAppSink' object has no attribute 'try_pull_sample'
from gi.repository import Gst, GLib, GstApp

Gst.init(None)

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface


class GiCameraFrameCapture(CameraFrameCaptureInterface):
    def __init__(self, frame_source, frame_size=(1920, 1080), image_buffer_size=2):
        super().__init__(frame_source=frame_source, frame_size=frame_size, image_buffer_size=image_buffer_size)
        self._running = True
        # Creating and setting the Gstreamer pipeline to the ready to use state
        self._gstPipeline = Gst.parse_launch(f"rtspsrc location={frame_source} latency=0 ! decodebin ! videoconvert ! video/x-raw, format=RGB ! appsink name=sink sync=false")
        self._appSink = self._gstPipeline.get_by_name("sink")
        self._gstPipeline.set_state(Gst.State.PLAYING)
        self._frameCaptureThread = Thread(target=self._capturing)
        self._frameCaptureThread.start()
        return

    def _capturing(self):
        context = GLib.MainContext.default()
        while self._running:
            try:
                # Run a single iteration of the default context
                while context.pending():
                    context.iteration(False)
                # Try to get a rawFrame: Gst.Sample
                rawFrame = self._appSink.try_pull_sample(Gst.SECOND)
                try:
                    frame = self._extractFrame(rawFrame)
                except RuntimeError as e:
                    rospy.loginfo(e)
                    frame = None
                except Exception as e:
                    rospy.loginfo(e)
                    frame = None

                if frame is None:
                    rospy.loginfo("Nothing captured!")
                    continue

                # Process the frame and add it to the image buffer
                processed_frame = self._process_frame(frame)
                self._addImageToBuffer(processed_frame)
            except Exception as e:
                print(e)
                break
        return

    def _extractFrame(self, rawFrame: Gst.Sample):
        # Get the actual data
        buffer = rawFrame.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            raise RuntimeError("Could not map buffer data!")

        numpy_frame = np.ndarray(
            shape=(self._frame_size[1], self._frame_size[0], 3),
            dtype=np.uint8,
            buffer=map_info.data)

        frame = np.copy(numpy_frame)

        # Clean up the buffer mapping
        buffer.unmap(map_info)
        return frame

    def _process_frame(self, frame):
        processed_frame = cv2.rectangle(frame, (50, 50), (200, 200), (0, 255, 0), 3)
        return processed_frame

    def stop(self):
        rospy.loginfo("Stopping the video capture thread...")
        self._running = False
        self._frameCaptureThread.join()
        self._gstPipeline.set_state(Gst.State.NULL)
        rospy.loginfo("Frame capture thread stopped.")
