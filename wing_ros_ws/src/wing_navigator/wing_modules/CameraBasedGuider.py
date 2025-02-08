import math

import rospy
from simple_pid import PID


class CameraBasedGuider(object):
    def __init__(self, x_pid_consts: list, y_pid_consts: list, throttle_profile: str,
                 throttle_sigmoid_profile_constants: list, const_throttle: float):
        """
        This class implement the PID based control loop for the final phase guider that it is using the camera as the
        sensor. Note that the feedback values, such as errors and bounding box size, are calculated in the reference
        zoom level. So, The caller of the loop_once method which is actually the feedback loop of the guider, must map
        the feedback values to the corresponding values in the reference zoom level(1X), so this class does not need to
        know about the zoom level of the camera.

        The throttle can be tuned by a sigmoid profile as below, when the target is not in near vicinity:

            throttle = 1.0 / (1 + math.exp(a * (self._objSize - b)))

        where the bigger 'a', bigger maximum throttle in profile and the bigger 'b', maximum throttle lasts in a bigger
        portion of the arrival path.
        also in the code of the profile, there is a size of object that we can assume we are very near to the
        destination when the object in the image is bigger than that size. When the target is in near vicinity,
        the throttle is set to a constant value based on the Y axis error in the input of the control loop. So, The Y
        axis error will be divided into 3 regions by providing 2 thresholds, which determines if the wing is too above,
        too below of the target or if the wing and the target are approximately at the same level.

        @param x_pid_consts: A list with 3 elements that define the PID constants for the X axis in camera frame.
        @param y_pid_consts: A list with 3 elements that define the PID constants for the Y axis in camera frame.
        @param throttle_profile: "constant" or "custom_sigmoid"
        @param throttle_sigmoid_profile_constants: A list with 8 elements that defines the sigmoid profile used as the
        throttle profile.
        @param const_throttle: A float between 0.0 and 1.0 that defines the constant throttle value used in the
        constant throttle profile. Value 0.5 is used in the simulation tests.
        """
        self._pixel_error = (0, 0)
        self._tg_size = 0
        self._throttle_profile = throttle_profile
        self._const_throttle = const_throttle
        self._sigmoid_constants = None
        self.set_throttle_sigmoid_profile_constants(throttle_sigmoid_profile_constants)
        self._x_pid_consts = x_pid_consts
        self._y_pid_consts = y_pid_consts
        self._x_pid = PID(*self._x_pid_consts)
        self._y_pid = PID(*self._y_pid_consts)
        return

    def loop_once(self, pixel_error: tuple, tg_size: int):
        """
        This function should be called in a loop to get the control output for the guider. Actually the loop is
        triggered by the detection and tracking on the camera frame.
        @param pixel_error: The X and Y errors of the target from the reference point in the image, in the reference
        zoom level. Note that the errors are determined as reference point - detected point. For example, error in x
        direction and reference zoom level is: ex = (x_ref - x_det)/zoom_level = (x_center - x_detected)/zoom_level.
        @param tg_size: Size of the bounding box of the target in the reference zoom level. This can be determined by
        BB_Area / zoom_level^2.
        @return: returns roll, pitch, yaw and throttle values that must be sent to the drone as AttitudeTarget command.
        """
        self._pixel_error = pixel_error
        self._tg_size = tg_size
        roll = math.radians(self._x_pid(self._pixel_error[0]))
        pitch = math.radians(self._y_pid(self._pixel_error[1]))
        yaw = 0.0
        throttle = self._get_throttle()
        return roll, pitch, yaw, throttle

    def reset(self):
        """
        Reset to not get a big control output in the start of a new approach duo to the accumulation of the integral
        parts of errors.
        """
        self._x_pid.reset()
        self._y_pid.reset()
        self._pixel_error = (0, 0)
        self._tg_size = 0
        return

    def set_x_pid_consts(self, x_pid_consts: list):
        self._x_pid_consts = x_pid_consts
        self._x_pid.tunings = self._x_pid_consts
        return

    def set_y_pid_consts(self, y_pid_consts: list):
        self._y_pid_consts = y_pid_consts
        self._y_pid.tunings = self._y_pid_consts
        return

    def set_throttle_profile(self, throttle_profile: str):
        self._throttle_profile = throttle_profile
        return

    def set_throttle_sigmoid_profile_constants(self, throttle_sigmoid_profile_constants: list):
        self._sigmoid_constants = {
            "a": throttle_sigmoid_profile_constants[0],
            "b": throttle_sigmoid_profile_constants[1],
            "size_threshold": throttle_sigmoid_profile_constants[2],
            "wing_too_below_threshold": throttle_sigmoid_profile_constants[3],
            "wing_too_below_throttle": throttle_sigmoid_profile_constants[4],
            "wing_too_above_threshold": throttle_sigmoid_profile_constants[5],
            "wing_too_above_throttle": throttle_sigmoid_profile_constants[6],
            "wing_tg_at_same_level_throttle": throttle_sigmoid_profile_constants[7]
        }
        return

    def set_const_throttle(self, const_throttle: float):
        self._const_throttle = const_throttle
        return

    def _get_throttle(self):
        """
        Get the throttle value based on the throttle profile.
        @return: 0.0 to 1.0 throttle value.
        """
        if "constant" == self._throttle_profile:
            throttle = self._const_throttle
        elif "customSigmoid" == self._throttle_profile:
            throttle = self._sigmoid_profile()
        else:
            rospy.logwarn(f"The profile {self._throttle_profile} is not supported. Falling back to constant throttle.")
            throttle = self._const_throttle
        return throttle

    def _sigmoid_profile(self):
        # TODO: clean this function in future! I think, the code is not readable for future developers.
        throttle = self._const_throttle
        # The loop is done when there is a new detection, so the target size would be greater than 0, otherwise is 0.
        if self._tg_size:
            y_axis_error = self._pixel_error[1]
            # If the target is in near vicinity.
            if self._tg_size > self._sigmoid_constants["size_threshold"]:
                # wing is too below the target
                if y_axis_error > self._sigmoid_constants["wing_too_below_threshold"]:
                    # throttle += 0.99 * (self._pixel_error[1] - 30)
                    throttle = self._sigmoid_constants["wing_too_below_throttle"]
                # wing is too above the target
                if y_axis_error < self._sigmoid_constants["wing_too_above_threshold"]:
                    throttle = self._sigmoid_constants["wing_too_above_throttle"]
                else:  # wing and target are approximately at the same level.
                    throttle = self._sigmoid_constants["wing_tg_at_same_level_throttle"]
            else:  # If the target is not in near vicinity, so use the sigmoid function for throttle.
                throttle = 1.0 / (
                        1 + math.exp(self._sigmoid_constants["a"] * (self._tg_size - self._sigmoid_constants["b"])))
        return throttle
