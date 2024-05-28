#!/usr/bin/env python

import math

import cv2
import numpy as np
import rospy
import tf.transformations
from cv_bridge import CvBridge
from mavros_msgs.msg import OverrideRCIn, AttitudeTarget
from sensor_msgs.msg import Image
from simple_pid import PID
from std_msgs.msg import Float32
import argparse


class interceptionController(object):
    def __init__(self, xPIDValues, xSetpoint, yPIDValues, ySetpoint, throttleProfileConstants, imageTopic,
                 attitudeCommandTopic, errorTopic, profileType, verbose=False):
        """
        The throttle profile is a customization of sigmoid function in the below form:
        throttle = 1.0 / (1 + math.exp(a * (self._objSize - b)))
        Bigger a bigger maximum throttle in profile
        Bigger b, maximum throttle lasts bigger portion of the path
        also in the code of the profile, there is a size of object that we can assume we are very near to the
        destination when the object in the image is bigger than that size.

        @param xPIDValues: list of PID values for horizontal/Roll controller
        @param yPIDValues: list of PID values for vertical/Pitch controller
        @param throttleProfileConstants: A list of throttle profile constants
        @param verbose: verbose mode of terminal logs
        """
        self._cvBridge = CvBridge()
        self._pixelError = (0, 0)
        self._objSize = None
        self._startFinalApproach = False
        self._profileType = profileType
        self._verbose = verbose

        # PID values that works for 0.5 throttle and the throttle profile
        self._pidX = PID(*xPIDValues, setpoint=xSetpoint)
        self._pidY = PID(*yPIDValues, setpoint=ySetpoint)
        self._throttleProfileConstants = throttleProfileConstants
        self._imageSub = rospy.Subscriber(imageTopic, Image, callback=self._imageCallback, queue_size=1)
        self._attitudePub = rospy.Publisher(attitudeCommandTopic, AttitudeTarget, queue_size=10)
        self._errorMagPub = rospy.Publisher(errorTopic, Float32, queue_size=10)
        return

    def __del__(self):
        cv2.destroyAllWindows()
        return

    def _imageCallback(self, msg):
        cv2Image = self._cvBridge.imgmsg_to_cv2(msg)
        width, height = cv2Image.shape[1], cv2Image.shape[0]
        center = (width // 2, height // 2)
        cx, cy, output_image = self._detectTarget(cv2Image)
        self._pixelError = (center[0] - cx, center[1] - cy)
        if self._verbose:
            rospy.loginfo(f"Error_x: {self._pixelError[0]}, Error_y: {self._pixelError[1]}")
        self._sendControlCommand()
        cv2.circle(output_image, center, 5, (0, 0, 255), -1)
        cv2.imshow('Filter Results', output_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("g"):
            # Reset to not get a big command in the start of controller for accumulation of the
            # integral part of error.
            self._pidX.reset()
            self._pidY.reset()
            self._objSize = None
            self._startFinalApproach = True
            rospy.loginfo(f"final approach controller is activated!")
        elif key == ord("s"):
            self._startFinalApproach = False
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
            self._objSize = cv2.contourArea(largest_contour)

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
        # self._rc_pub.publish(rc_override)
        return

    def _send_attitude_command(self, roll, pitch, yaw, throttle):
        attitude_target = AttitudeTarget()
        attitude_target.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        attitude_target.orientation.x = quaternion[0]
        attitude_target.orientation.y = quaternion[1]
        attitude_target.orientation.z = quaternion[2]
        attitude_target.orientation.w = quaternion[3]
        attitude_target.thrust = throttle

        self._attitudePub.publish(attitude_target)
        return

    def _sendControlCommand(self):
        errorMagnitude = math.sqrt(math.pow(self._pixelError[0], 2) + math.pow(self._pixelError[1], 2))
        self._errorMagPub.publish(errorMagnitude)
        if self._startFinalApproach:
            roll = math.radians(self._pidX(self._pixelError[0]))  # Example roll value in radians
            pitch = math.radians(self._pidY(self._pixelError[1]))
            yaw = 0.0  # Example yaw value in radians
            throttle = self._getThrust()
            self._send_attitude_command(roll, pitch, yaw, throttle)
        return

    def _getThrust(self):
        if "constant" == self._profileType:
            throttle = 0.5
        elif "customSigmoid" == self._profileType:
            throttle = self._sigmoidThrottleProfile()
        else:
            throttle = 0.5
        return throttle

    def _sigmoidThrottleProfile(self):
        throttle = 0.5
        if self._objSize is not None:
            if self._verbose:
                rospy.loginfo(f"Object contour size: {self._objSize}.")
            throttle = 1.0 / (1 + math.exp(
                self._throttleProfileConstants[0] * (self._objSize - self._throttleProfileConstants[1])))
            if self._objSize > self._throttleProfileConstants[2]:
                # wing is below the target
                if self._pixelError[1] > 30:
                    # throttle += 0.99 * (self._error[1] - 30)
                    throttle = 0.65
                # wing is above the target
                if self._pixelError[1] < -30:
                    throttle = 0.3
                else:
                    throttle = 0.45
        return throttle


class ParseList(argparse.Action):
    """
    A class for getting list arguments from command line like below:
    sampleScript.py --pid "[1 2 3]"
    """

    def __call__(self, parser, namespace, values, option_string=None):
        pid_list = [float(x) for x in values.strip('[]').split()]
        setattr(namespace, self.dest, pid_list)


def main():
    # Initialize the ROS node
    rospy.init_node('send_roll_pitch', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', default=False)
    parser.add_argument('--profileType', default="constant")  # or customSigmoid
    parser.add_argument('--profileConstants', action=ParseList, default=[0.003, 850, 917])
    parser.add_argument('--xPIDValues', action=ParseList, default=[0.045, 0, 0.01])
    parser.add_argument("--xSetpoint", default=0.0)
    parser.add_argument('--yPIDValues', action=ParseList, default=[0.12, 0, 0.01])
    parser.add_argument("--ySetpoint", default=0.0)
    parser.add_argument("--attitudeCommandTopic", default='/mavros/setpoint_raw/attitude')
    parser.add_argument("--errorTopic", default='/funnywing/lastPhaseControllerError')
    parser.add_argument("--imageTopic", default='/front_camera_ns/image_raw')
    args = parser.parse_args()
    lastPhaseController = interceptionController(args.xPIDValues, args.xSetpoint, args.yPIDValues, args.ySetpoint,
                                                 args.profileConstants, args.imageTopic, args.attitudeCommandTopic,
                                                 args.errorTopic, args.profileType, args.verbose)
    rospy.spin()
    del lastPhaseController
    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
