from threading import Thread

import cv2
import ffmpeg
import numpy as np
import rospy

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface


class FfmpegCameraFrameCapture(CameraFrameCaptureInterface):
    def __init__(self, frame_source, frame_size=(1920, 1080), image_buffer_size=2):
        super().__init__(frame_source=frame_source, frame_size=frame_size, image_buffer_size=image_buffer_size)
        self._running = True
        self._frame_source = frame_source
        # Initialize the FFmpeg process to read from the RTSP stream
        self._ffmpeg_process = (
            ffmpeg
            # remove low_delay flag if you want more image quality in terms of a little delay.
            .input(self._frame_source, fflags='nobuffer', flags='low_delay')
            .output('pipe:', format='rawvideo', pix_fmt='bgr24', s=f'{self._frame_size[0]}x{self._frame_size[1]}')
            # I think this is non-blocking, so we can create the process here, otherwise it should be initialized
            # in the self._capturing method.
            .run_async(pipe_stdout=True)
        )
        self._frameCaptureThread = Thread(target=self._capturing)
        self._frameCaptureThread.start()
        return

    def _capturing(self):
        while self._running:
            try:
                # Read raw frame data from the process
                in_bytes = self._ffmpeg_process.stdout.read(self._frame_size[0] * self._frame_size[1] * 3)
                if not in_bytes:
                    # If in future you have problem of breaking the capture loop, may be using continue instead of
                    # break would help.
                    break
                # Convert the bytes to a numpy array and reshape to an image
                frame = np.frombuffer(in_bytes, np.uint8).reshape([self._frame_size[1], self._frame_size[0], 3])
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
        self._ffmpeg_process.stdout.close()
        self._ffmpeg_process.terminate()
        self._ffmpeg_process.wait()
        rospy.loginfo("Frame capture thread stopped.")
