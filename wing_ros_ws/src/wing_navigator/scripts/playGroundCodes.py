#!/usr/bin/env python

#######################################################################################################################
#   TODO: This is a file to write try/error things and so on. Don't leave any valuable code here, since it will be
#    deleted and Transfer valuable codes to major files of the Project.
######################################################################################################################

import rospy
from mavros_msgs.msg import OverrideRCIn, RCIn
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from pynput import keyboard

# from ultralytics import YOLO
# from rospkg import RosPack as rospack
#
# model = YOLO(rospack().get_path("wing_navigator") + "/scripts/objectDetectionModels/yolov8x.pt")

global final_approach_state
final_approach_state = False


# def imageCallback(msg):
#     bridge = CvBridge()
#     cv2Image = bridge.imgmsg_to_cv2(msg)
#     results = model.predict(cv2Image)
#     annotatedFrame = results[0].plot()
#     # cv2.imshow('front camera', cv2Image)
#     cv2.imshow('Inference Results', annotatedFrame)
#     cv2.waitKey(1)
#     return

def detectTarget(cv2Image):
    grayImage = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2GRAY)
    output_image = cv2.cvtColor(grayImage, cv2.COLOR_GRAY2BGR)
    _, thresholded_image = cv2.threshold(grayImage, 170, 255, cv2.THRESH_BINARY_INV)
    kernel_size = (7, 7)  # Adjust the size as needed
    kernel = np.ones(kernel_size, np.uint8)
    dilated_image = cv2.dilate(thresholded_image, kernel, iterations=3)
    # Find contours in the eroded image
    contours, _ = cv2.findContours(dilated_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Ignore contours that are too close to the image borders
    filtered_contours = []
    height, width = dilated_image.shape
    border_margin = 5  # Margin from the border to consider a contour as being part of the border
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if x > border_margin and y > border_margin and (x + w) < (width - border_margin) and (y + h) < (
                height - border_margin):
            filtered_contours.append(contour)
    # Ensure that there is at least one contour found
    cX, cY = 0, 0
    if filtered_contours:
        # Find the largest contour
        largest_contour = max(filtered_contours, key=cv2.contourArea)

        # Calculate the moments of the largest contour
        M = cv2.moments(largest_contour)

        # Calculate the centroid (center of mass) of the largest contour
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # Draw the centroid on the original grayscale image
        cv2.circle(output_image, (cX, cY), 5, (0, 255, 0), -1)
    return cX, cY, output_image


def setFinalApproachState():
    return


def imageCallback(msg):
    bridge = CvBridge()
    cv2Image = bridge.imgmsg_to_cv2(msg)
    width, height = cv2Image.shape[1], cv2Image.shape[0]
    center = (width // 2, height // 2)
    cx, cy, output_image = detectTarget(cv2Image)
    e = (center[0] - cx, center[1] - cy)
    rospy.loginfo(f"ex: {e[0]}, ey: {e[1]}")
    cv2.circle(output_image, center, 5, (0, 0, 255), -1)
    cv2.imshow('Filter Results', output_image)
    key = cv2.waitKey(1) & 0xFF
    global final_approach_state
    if key == ord("g"):
        final_approach_state = True
    elif key == ord("s"):
        final_approach_state = False
    return


def on_press(key):
    global final_approach_state
    if key.char == 'g':
        final_approach_state = True
        rospy.loginfo("Got key g")
    elif key.char == 's':
        final_approach_state = False
        rospy.loginfo("Got key s")
    return


if "__main__" == __name__:
    rospy.init_node("playGroundCodes", anonymous=True)
    # image_sub = rospy.Subscriber("/front_camera_ns/image_raw", Image, callback=imageCallback)
    # rospy.spin()
    # cv2.destroyAllWindows()
    # Create a publisher for the RC override topic

    rc_override_pub = rospy.Publisher('/mavros/rc/in', RCIn, queue_size=10)
    # Set up the listener for key press events
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Set the rate to publish the RC override values
    rate = rospy.Rate(10)  # 10 Hz

    # Create an OverrideRCIn message instance
    # rc_msg = OverrideRCIn()
    rc_msg = RCIn()
    rc_msg.channels = [1300, 1347, 1500, 1500, 1500, 1500, 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0]
    # Continuously publish the RC override values
    while not rospy.is_shutdown():
        if final_approach_state:
            rc_override_pub.publish(rc_msg)
        rate.sleep()

    listener.stop()
