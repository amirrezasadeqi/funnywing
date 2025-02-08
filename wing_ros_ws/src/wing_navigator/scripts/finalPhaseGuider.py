#!/usr/bin/env python3
import rospy
import tf.transformations
from mavros_msgs.msg import AttitudeTarget
from PySide2.QtCore import QObject, Slot

from wing_modules.CameraBasedGuider import CameraBasedGuider
from wing_modules.CameraBasedGuiderSensorBlock import CameraBasedGuiderSensorBlock
from wing_modules.CameraInterface.CameraCaptureInterfaceImplementation.RosImageTopicCameraFrameCapture import \
    RosImageTopicCameraFrameCapture


class FinalPhaseGuider(QObject):
    def __init__(self, cam_based_guider: CameraBasedGuider, cam_based_sensor_block: CameraBasedGuiderSensorBlock,
                 attitudeCommandTopic: str):
        super().__init__()
        self._attitude_pub = rospy.Publisher(attitudeCommandTopic, AttitudeTarget, queue_size=1)
        self._cam_based_guider = cam_based_guider
        self._cam_based_sensor_block = cam_based_sensor_block
        # When track is locked, the guidance loop will be triggered on each track update
        self._cam_based_sensor_block.trigger_guidance_loop.connect(self.on_trigger_guidance_loop)
        # When track is lost, the guidance PIDs will be reset to prevent wierd behaviors on the next track lock duo to
        # the accumulation of the integral parts of errors.
        self._cam_based_sensor_block.track_lost.connect(self.reset_guidance)
        return

    @Slot(tuple, int)
    def on_trigger_guidance_loop(self, pixel_error, bb_area_feedback):
        """
        The pixel error and bb_area feedbacks are in the reference zoom level.
        """
        # loop the guidance and get the attitude command
        roll, pitch, yaw, throttle = self._cam_based_guider.loop_once(pixel_error, bb_area_feedback)
        # send the attitude command to the drone
        self._send_attitude_command(roll, pitch, yaw, throttle)
        return

    @Slot()
    def reset_guidance(self):
        self._cam_based_guider.reset()
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

        self._attitude_pub.publish(attitude_target)
        return


def main():
    rospy.init_node('final_phase_guider', anonymous=True)
    guider = CameraBasedGuider([0.045, 0, 0.01], [0.12, 0, 0.01], "constant",
                               [0.003, 850.0, 917, 30, 0.65, -30, 0.3, 0.45], 0.5)
    frame_capture = RosImageTopicCameraFrameCapture("/front_camera_ns/image_raw")
    cam_based_sensor_block = CameraBasedGuiderSensorBlock(frame_capture, detection_model_file="funnyYolo100K8m.pt",
                                                          frame_size=(1920, 1080))
    final_phase_guider = FinalPhaseGuider(guider, cam_based_sensor_block, "/mavros/setpoint_raw/attitude")
    rospy.spin()
    return


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down final phase guider node by keyboard interrupt!")
