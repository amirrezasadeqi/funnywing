from dronekit import LocationGlobalRelative, connect, VehicleMode
import rospy
import time
from wing_navigator.srv import SimpleGoto, SimpleGotoResponse, ActiveMode, ActiveModeResponse, ArmTakeoff, ArmTakeoffResponse
                               

class navigator:
    """A Class which contains all the methods for navigation purpose"""

    def __init__(self, connection_string):
        print("Connecting to vehicle on: %s" % connection_string)
        self.vehicle = connect(connection_string, wait_ready=True)
        self.goto_server = rospy.Service("/simple_goto", SimpleGoto, self.simple_goto_handler)
        print("Simple goto is at your service!")
        self.active_mode_server = rospy.Service("/active_mode", ActiveMode, self.active_mode_handler)
        print("Activation Mode Server is at your service!")
        self.arm_takeoff_server = rospy.Service("/arm_takeoff", ArmTakeoff, self.arm_takeoff_handler)
        print("Arm and Takeoff Server is at your service!")

    def arm_takeoff_handler(self, req):
        resp = ArmTakeoffResponse()
        try:
            self.vehicle_arm()
            resp.accepted = True
            return resp
        except:
            resp.accepted = False
            return resp

    def active_mode_handler(self, req):
        resp = ActiveModeResponse()
        try:
            self.vehicle.mode = VehicleMode(req.mode)
            print(f"Flight Mode has been changed to {req.mode}")
            resp.accepted = True
            return resp
        except:
            resp.accepted = False
            return resp

    def simple_goto_handler(self, req):
        resp = SimpleGotoResponse()
        try:
            print("Going to your desired position")
            self.vehicle.mode = VehicleMode("GUIDED")
            goto_location = LocationGlobalRelative(req.lat, req.lon, req.alt)
            self.vehicle.simple_goto(goto_location)
            resp.accepted = True
            return resp
        except:
            resp.accepted = False
            return resp

    def vehicle_arm(self):
      """
        Safely Arm the vehicle!
      """

      print("Change Mode to AUTO")
      self.vehicle.mode = VehicleMode("TAKEOFF")

      print("Wait for Pre-flight Checks!")
      while not self.vehicle.is_armable:
        print("Wait for Consistent Sensor Datas and Pre-flight checks!")
        time.sleep(1)

      print("Arming Motors!")
      self.vehicle.arm()

