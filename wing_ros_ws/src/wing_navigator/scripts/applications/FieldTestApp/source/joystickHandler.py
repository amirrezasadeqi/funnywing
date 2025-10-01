#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn
import threading

class JoystickHandler:
    def __init__(self,
                 joy_topic="/joy",
                 output_topic="/GCS/from"):

        rospy.init_node("joystick_handler_node", anonymous=True)

        # Publisher به تاپیک GCS
        self.pub = rospy.Publisher(output_topic, OverrideRCIn, queue_size=10)

        # متغیرهای جوی‌استیک
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw = 0

        # Subscriber جوی‌استیک
        rospy.Subscriber(joy_topic, Joy, self.joy_callback)

        # Thread برای ارسال velocity
        self.running = True
        self.thread = threading.Thread(target=self.send_velocity_loop)
        self.thread.start()

        rospy.loginfo(f"🎮 JoystickHandler initialized. Publishing to: {output_topic}")

    def joy_callback(self, msg):
        """
        تبدیل ورودی جوی‌استیک به vx, vy, vz, yaw
        فرض شده:
        - Left stick: x->vx, y->vy
        - Right stick y->vz, x->yaw
        مقادیر بین -1 تا 1
        """
        self.vx = msg.axes[1] * 2.0  # سرعت جلو/عقب
        self.vy = msg.axes[0] * 2.0  # سرعت چپ/راست
        self.vz = msg.axes[3] * 1.0  # سرعت بالا/پایین
        self.yaw = msg.axes[2] * 1.5  # yaw rate

    def send_velocity_loop(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and self.running:
            # ایجاد پیام OverrideRCIn و تبدیل مقادیر جوی‌استیک
            rc_msg = OverrideRCIn()
            rc_msg.channels = [
                int(1500 + self.vx * 500), # Roll
                int(1500 + self.vy * 500), # Pitch
                int(1500 + self.vz * 500), # Throttle
                int(1500 + self.yaw * 500), # Yaw
                0, 0, 0, 0
            ]
            self.pub.publish(rc_msg)
            rate.sleep()

    def stop(self):
        self.running = False
        self.thread.join()

if __name__ == "__main__":
    handler = JoystickHandler()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        handler.stop()
