import cv2
import rospy
from wing_navigator.msg import Track
from PySide2.QtCore import QObject


class FrameProcessor(QObject):
    def __init__(self, track_topic):
        """
        _tracks: [
            {
            "frame_count": frame_count,
            "tracks": [{
                "track_id": track_id,
                "track_locked": track_locked,
                "bounding_box": [(x0, y0), (x1, y1)]
                }]
            }
        ]
        """
        super().__init__()
        # TODO: It may be necessary to tune the queue_size of the subscriber to not drop tracks or to not have delay.
        self._track_sub = rospy.Subscriber(track_topic, Track, callback=self._track_sub_callback)
        # list of dictionaries. Each dictionary contains tracks corresponding to a same frame.
        self._tracks = []
        self._backFrontConnection = None
        return

    def setQtCommunicator(self, backFrontConnection):
        self._backFrontConnection = backFrontConnection
        return

    def _track_sub_callback(self, msg: Track):
        if len(self._tracks) and (msg.frame_count == self._tracks[-1]["frame_count"]):
            self._append_track_to_last_frame(msg)
        else:
            self._append_new_frame_of_tracks(msg)
        return

    def process_frame(self, frame):
        """
        @param frame: frames must be in RGB format. For example, you can use cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            to convert the frame before feeding it to this method.
        @return: returns the processed frame in which the corresponding tracks are drawn.
        """
        width, height = frame.shape[1], frame.shape[0]
        # Get the tracks corresponding to the last frame. In the future, we may need to ignore outdated tracks.
        if len(self._tracks):
            # !Note! this is not the last frame actually, but for now we don't have any sense about the synchronization
            # state and in future we may make the code here better.
            last_frame = self._tracks.pop(0)
            last_track_id = -1
            for track in last_frame["tracks"]:
                color = (255, 0, 0) if track["track_locked"] else (0, 255, 0)
                x0, y0 = int(track["bounding_box"][0][0] * width), int(track["bounding_box"][0][1] * height)
                x1, y1 = int(track["bounding_box"][1][0] * width), int(track["bounding_box"][1][1] * height)
                frame = cv2.rectangle(frame, (x0, y0), (x1, y1), color, 3)
                frame = cv2.putText(frame, f"id:{track['track_id']}", (x0, y0 - 5), cv2.FONT_HERSHEY_COMPLEX,
                                    fontScale=2, color=color, thickness=5)
                # determining the more recent track id, since it is usually the interested
                if track['track_id'] > last_track_id:
                    last_track_id = track['track_id']
            # update the last track id in the back-end to be able to lock on it.
            self._emit_last_track_id(last_track_id)
        frame = cv2.drawMarker(frame, (width // 2, height // 2), (255, 0, 0), markerType=cv2.MARKER_CROSS,
                               markerSize=50, thickness=2, line_type=cv2.LINE_AA)
        processed_frame = frame
        return processed_frame

    def _append_track_to_last_frame(self, track: Track):
        self._tracks[-1]["tracks"].append({
            "track_id": track.track_id,
            "track_locked": track.track_state,
            "bounding_box": [(track.top_left_x, track.top_left_y), (track.bottom_right_x, track.bottom_right_y)]
        })
        return

    def _append_new_frame_of_tracks(self, track: Track):
        self._tracks.append({
            "frame_count": track.frame_count,
            "tracks": [{
                "track_id": track.track_id,
                "track_locked": track.track_state,
                "bounding_box": [(track.top_left_x, track.top_left_y), (track.bottom_right_x, track.bottom_right_y)]
            }]
        })
        return

    def _emit_last_track_id(self, last_track_id):
        try:
            self._backFrontConnection.setLastTrackIdSignal.emit(last_track_id)
        except Exception as e:
            rospy.logwarn(
                f"Exception occurred on sending last_track_id to the backend. Maybe the _backFrontConnection is not set"
                f" yet: {e}")
        return
