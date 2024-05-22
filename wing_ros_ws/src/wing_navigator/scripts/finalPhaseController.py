#!/usr/bin/env python

import random

import rospy
from mavros_msgs.msg import OverrideRCIn, AttitudeTarget
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import tf.transformations


class interceptionController(object):
    def __init__(self):
        image_sub = rospy.Subscriber("/front_camera_ns/image_raw", Image, callback=self._imageCallback)
        self._bridge = CvBridge()
        self._error = (0, 0)
        self._kpx = -0.05
        self._kpy = -0.12
        self._final_approach_state = False
        self._attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        return

    def __del__(self):
        cv2.destroyAllWindows()
        return

    def _imageCallback(self, msg):
        cv2Image = self._bridge.imgmsg_to_cv2(msg)
        width, height = cv2Image.shape[1], cv2Image.shape[0]
        center = (width // 2, height // 2)
        cx, cy, output_image = self._detectTarget(cv2Image)
        self._error = (center[0] - cx, center[1] - cy)
        rospy.loginfo(f"error_x: {self._error[0]}, error_y: {self._error[1]}")
        self._sendControlCommand()
        cv2.circle(output_image, center, 5, (0, 0, 255), -1)
        cv2.imshow('Filter Results', output_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("g"):
            self._final_approach_state = True
            rospy.loginfo(f"final approach controller is activated!")
        elif key == ord("s"):
            self._final_approach_state = False
            rospy.loginfo(f"final approach controller is deactivated!")
        return

    def _detectTarget(self, cv2Image):
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

    def _send_rc_override(self, roll, pitch, throttle, yaw):
        rc_override = OverrideRCIn()
        # Map roll, pitch, throttle, yaw to corresponding RC channels
        rc_override.channels[0] = roll  # Roll (usually channel 1)
        rc_override.channels[1] = pitch  # Pitch (usually channel 2)
        rc_override.channels[2] = throttle  # Throttle (usually channel 3)
        rc_override.channels[3] = yaw  # Yaw (usually channel 4)
        # Other channels can be set to 0 (no override) or any desired value
        rc_override.channels[4] = 0
        rc_override.channels[5] = 0
        rc_override.channels[6] = 0
        rc_override.channels[7] = 0
        # TODO
        # rc_pub.publish(rc_override)
        return

    def _send_attitude_command(self, roll, pitch, yaw, thrust):
        attitude_target = AttitudeTarget()
        attitude_target.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        attitude_target.orientation.x = quaternion[0]
        attitude_target.orientation.y = quaternion[1]
        attitude_target.orientation.z = quaternion[2]
        attitude_target.orientation.w = quaternion[3]
        attitude_target.thrust = thrust

        self._attitude_pub.publish(attitude_target)
        return

    def _sendControlCommand(self):
        roll = math.radians(self._kpx * self._error[0])  # Example roll value in radians
        # roll = math.radians(0)  # Example roll value in radians
        pitch = math.radians(self._kpy * self._error[1])  # Example pitch value in radians
        yaw = 0.0  # Example yaw value in radians
        thrust = 0.5  # Example thrust value (0 to 1)
        if self._final_approach_state:
            self._send_attitude_command(roll, pitch, yaw, thrust)
        return

# def send_rc_override(roll, pitch, throttle, yaw):
#     rc_override = OverrideRCIn()
#     # Map roll, pitch, throttle, yaw to corresponding RC channels
#     rc_override.channels[0] = roll  # Roll (usually channel 1)
#     rc_override.channels[1] = pitch  # Pitch (usually channel 2)
#     rc_override.channels[2] = throttle  # Throttle (usually channel 3)
#     rc_override.channels[3] = yaw  # Yaw (usually channel 4)
#     # Other channels can be set to 0 (no override) or any desired value
#     rc_override.channels[4] = 0
#     rc_override.channels[5] = 0
#     rc_override.channels[6] = 0
#     rc_override.channels[7] = 0
#     rc_pub.publish(rc_override)
#     return
#
#
# def send_attitude_command(roll, pitch, yaw, thrust):
#     attitude_target = AttitudeTarget()
#     attitude_target.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE
#     quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
#     attitude_target.orientation.x = quaternion[0]
#     attitude_target.orientation.y = quaternion[1]
#     attitude_target.orientation.z = quaternion[2]
#     attitude_target.orientation.w = quaternion[3]
#     attitude_target.thrust = thrust
#
#     attitude_pub.publish(attitude_target)
#     return


def main():
    # global rc_pub
    # global attitude_pub

    # Initialize the ROS node
    rospy.init_node('send_roll_pitch', anonymous=True)

    # Create a publisher for the RC override topic
    # rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    # create a publisher for sending attitude setpoints
    # attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

    # Set the rate to publish the RC override values
    # rate = rospy.Rate(20)  # 20 Hz

    lastPhaseController = interceptionController()
    rospy.spin()

    # Main loop
    # while not rospy.is_shutdown():
    #     # roll = 1500  # Neutral roll
    #     # pitch = 1500  # Neutral pitch
    #     # throttle = 1500  # Neutral throttle
    #     # yaw = 1500  # Neutral yaw
    #     #
    #     # # Example: Roll left
    #     # # roll = 1500
    #     # roll = random.randint(-400, 400) + 1500
    #     # # Example: Pitch down
    #     # # pitch = 1800
    #     # pitch = random.randint(0, 400) + 1500
    #     # throttle = random.randint(0, 500) + 1500
    #     #
    #     # send_rc_override(roll, pitch, throttle, yaw)
    #     #
    #     # rate.sleep()
    #     roll = math.radians(random.randint(-20, 20))  # Example roll value in radians
    #     pitch = math.radians(random.randint(-15, 15))  # Example pitch value in radians
    #     yaw = 0.0  # Example yaw value in radians
    #     thrust = random.random()  # Example thrust value (0 to 1)
    #
    #     send_attitude_command(roll, pitch, yaw, thrust)
    #     rate.sleep()

    del lastPhaseController
    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
