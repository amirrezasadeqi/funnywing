import threading

import numpy as np
import rospy
from PySide2.QtCore import QObject, Signal, Slot
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from norfair import Detection, Tracker
from pymavlink import mavutil
from rospkg import RosPack as rospack
from ultralytics import YOLO
from wing_navigator.srv import LockOnOff, LockOnOffRequest, LockOnOffResponse

from wing_modules.CameraInterface.CameraFrameCaptureInterface import CameraFrameCaptureInterface


class CameraBasedGuiderSensorBlock(QObject):
    track_lost = Signal()
    trigger_guidance_loop = Signal(tuple, int)

    def __init__(self, frame_capture: CameraFrameCaptureInterface, detection_model_file, frame_size):
        """
        @param frame_capture: 
        @param detection_model_file: name of the model file, e.g. Yolov8m.pt, in the
        '<wing_navigator_pkg>/scripts/objectDetectionModels/' directory.
        @param frame_size: 
        """
        super().__init__()
        # TODO: encapsulate the detection model into the tracker interface.
        self._detection_model = YOLO(
            rospack().get_path("wing_navigator") + f"/scripts/objectDetectionModels/{detection_model_file}")
        # TODO: use tracker interface instead of using the norfair tracker directly
        self._tracker = Tracker(distance_function="euclidean", distance_threshold=100, initialization_delay=1,
                                hit_counter_max=400)
        self._frame_capture = frame_capture
        self._frame_size = frame_size
        self._point_ref = (frame_size[0] // 2, frame_size[1] // 2)
        self._lock_on_track = False
        self._locked_track_id = 0
        self._lock_on_off_service = rospy.Service("/funnywing/lock_on_off", LockOnOff,
                                                  self._lock_on_off_service_handler)
        # TODO: This should be updated on the change of camera zoom level
        self._zoom_level = 1
        self._track_sender_running = True
        self._track_sender_buffer = []
        self._track_sender_thread = threading.Thread(target=self._send_tracks_to_gcs)
        # Connecting the new frame signal to the methods doing the sensing after all the initializations are done.
        self._frame_capture.newFrameCaptured.connect(self.on_new_frame_captured)
        self._track_sender_thread.start()
        return

    def stop(self):
        self._track_sender_running = False
        self._track_sender_thread.join()
        self._frame_capture.stop()
        return

    @Slot()
    def on_new_frame_captured(self):
        # get the frame from the frame capture interface
        frame = self._frame_capture.get_frame()
        if frame is not None:
            # run the detection model on the frame
            detections = self._detection_model.predict(frame, save=False, verbose=False)[0]
            # convert the detections to the input of traker
            norfair_detections = self._ultralytics_to_norfair_detections(detections)
            # update the tracks using the tracker
            tracked_objects = self._tracker.update(detections=norfair_detections)
            locked_track_flag = False
            if self._lock_on_track:
                if self._is_locked_track_still_available(tracked_objects):
                    # flag the track as locked
                    locked_track_flag = True
                    pixel_error, bb_area_feedback = self._get_feedback_values(tracked_objects)
                    # Signal the final phase guider to loop the guidance. the signal contains the feedback values.
                    self.trigger_guidance_loop.emit(pixel_error, int(bb_area_feedback))
                else:
                    # Interested track is lost, so disable the triggering of the guidance loop.
                    self._lock_on_track = False
                    # Signal the final phase guider to reset the guidance PIDs
                    self.track_lost.emit()
            # send the tracks to the GCS regardless of the track lock status. This is done by adding the tracks to a
            # buffer and send them by another thread to prevent delay in the guidance loop.
            self._track_sender_buffer.append({
                "tracks": tracked_objects,
                "lock_flag": locked_track_flag,
                "locked_track_id": self._locked_track_id
            })
        return

    def _is_locked_track_still_available(self, tracked_objects):
        # TODO: I think this should be encapsulated into the tracker interface
        for track in tracked_objects:
            if track.id == self._locked_track_id:
                return True
        return False

    def _get_locked_track(self, tracked_objects):
        # TODO: I think this should be encapsulated into the tracker interface
        for track in tracked_objects:
            if track.id == self._locked_track_id:
                return track
        return None

    def _get_feedback_values(self, tracked_objects):
        """
        This method calculates the feedback values for the guidance loop of the final phase guider. The feedback values
        are the pixel error and the bounding box area feedback at the reference zoom level.
        @param tracked_objects:
        @return:
        """
        # This code comes from the codes in the drawer.py file in the norfair repository.
        locked_track_box = tuple(self._get_locked_track(tracked_objects).estimate.astype(int))
        top_left, bottom_right = tuple(locked_track_box[0]), tuple(locked_track_box[1])
        cx, cy, bb_area = (top_left[0] + bottom_right[0]) // 2, (top_left[1] + bottom_right[1]) // 2, (
                bottom_right[0] - top_left[0]) * (bottom_right[1] - top_left[1])
        pixel_error_at_ref_zoom_level = (
            (self._point_ref[0] - cx) / self._zoom_level, (self._point_ref[1] - cy) / self._zoom_level)
        bb_area_feedback_at_ref_zoom_level = bb_area / (self._zoom_level ** 2)
        return pixel_error_at_ref_zoom_level, bb_area_feedback_at_ref_zoom_level

    def _ultralytics_to_norfair_detections(self, detections):
        """
        TODO: encapsulate this method into the tracker interface
        @param detections:
        @return:
        """
        norfair_detections = []
        boxes = detections.boxes.xywh.tolist()
        confidences = detections.boxes.conf.tolist()
        classes = detections.boxes.cls.tolist()
        for detection in zip(boxes, confidences, classes):
            x, y, w, h = detection[0]
            # Points to track are the points of the bounding box and I think we track two logically seperated points!
            points = np.array(
                [
                    [int(x - w // 2), int(y - h // 2)],
                    [int(x + w // 2), int(y + h // 2)]
                ]
            )
            # since we want to track bounding boxes, so we have two points to track with the same score, and score would
            # have two elements for each bounding box, not one and if you use one, there will be some dimension errors.
            scores = np.array([detection[1], detection[1]])
            label = int(detection[2])
            norfair_detections.append(
                Detection(
                    points=points,
                    scores=scores,
                    label=label
                ))
        return norfair_detections

    def _send_tracks_to_gcs(self):
        # the code only runs on the wing/RPI side.
        protocol_obj = mavutil.mavlink.MAVLink('', mavutil.mavlink.MAV_TYPE_FIXED_WING, 1)
        # publisher for sending the tracks to the GCS via embedding them into the messages coming from
        # autopilot.
        track_to_gcs_publisher = rospy.Publisher("/mavlink/from", Mavlink, queue_size=3)
        frame_count = 0
        while self._track_sender_running:
            if len(self._track_sender_buffer):
                frame_count += 1
                frame_tracks = self._track_sender_buffer.pop(0)
                for track in frame_tracks["tracks"]:
                    # checking for the case if we locked on the track on which we had to lock.
                    track_locked = True if frame_tracks["lock_flag"] and frame_tracks[
                        "locked_track_id"] == track.id else False
                    # creating the corresponding message for each track and send them to the GCS.
                    self._create_and_send_track_message(frame_count, track, track_locked, protocol_obj,
                                                        track_to_gcs_publisher)
            else:
                rospy.sleep(0.05)
        return

    def _create_and_send_track_message(self, frame_count, track, track_locked, protocol_obj, track_to_gcs_publisher):

        track_box = tuple(track.estimate.astype(int))
        top_left, bottom_right = tuple(track_box[0]), tuple(track_box[1])
        width, height = self._frame_size

        mavMsgFields = [
            int(rospy.Time.now().secs),
            track.id,
            frame_count,
            mavutil.mavlink.TRACK_STATE_LOCKED if track_locked else mavutil.mavlink.TRACK_STATE_UNLOCKED,
            float(top_left[0] / width),
            float(top_left[1] / height),
            float(bottom_right[0] / width),
            float(bottom_right[1] / height)
        ]

        mavMsg = mavutil.mavlink.MAVLink_track_status_message(*mavMsgFields)
        mavMsg.pack(protocol_obj)
        rosMsg = mavlink.convert_to_rosmsg(mavMsg)
        track_to_gcs_publisher.publish(rosMsg)
        return

    def _lock_on_off_service_handler(self, request: LockOnOffRequest):
        if request.lock_on:
            self._lock_on_track = True
            self._locked_track_id = request.track_id
        else:
            self._lock_on_track = False
            self._locked_track_id = 0
            # To reset the guider PIDs for the next track lock
            self.track_lost.emit()
        return LockOnOffResponse(True)
