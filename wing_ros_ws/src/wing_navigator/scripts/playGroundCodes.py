#!/usr/bin/env python

#######################################################################################################################
#   TODO: This is a file to write try/error things and so on. Don't leave any valuable code here, since it will be
#    deleted and Transfer valuable codes to major files of the Project.
######################################################################################################################

import rospy
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
from rospkg import RosPack as rospack
import cv2


class YOLOTestTracker(object):
    def __init__(self):
        self._model = YOLO(rospack().get_path("wing_navigator") + "/scripts/objectDetectionModels/funnyYolo100K8m.pt")
        self._bridge = CvBridge()
        rospy.Subscriber("/front_camera_ns/image_raw", Image, self._imageCallback)
        self._cnt = 0
        return

    def _imageCallback(self, msg):
        self._cnt += 1
        cv2Image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self._model.track(cv2Image, persist=True, save=False, verbose=False)
        # Visualize the results on the frame
        for result in results:
            try:
                rospy.loginfo(
                    f"Number of tracks in frame counter: {self._cnt} is: {len(result.boxes.id)} and the IDs are: {result.boxes.id}")
            except:
                pass

        # Display the annotated frame
        # cv2.imshow("YOLO11 Tracking", annotated_frame)
        #
        # # Break the loop if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     return
        return


def main():
    tracker = YOLOTestTracker()
    rospy.spin()
    return


if "__main__" == __name__:
    rospy.init_node("playGroundCodes", anonymous=True)
    main()
