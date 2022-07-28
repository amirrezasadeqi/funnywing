from dronekit import LocationGlobalRelative, connect, VehicleMode
import rospy
from wing_navigator.srv import SimpleGoto, SimpleGotoResponse

class navigator:
    """A Class which contains all the methods for navigation purpose"""

    def __init__(self, connection_string):
        print("Connecting to vehicle on: %s" % connection_string)
        self.vehicle = connect(connection_string, wait_ready=True)
        self.goto_server = rospy.Service("/simple_goto", SimpleGoto, self.simple_goto_handler)
        print("Simple goto is at your service!")

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
