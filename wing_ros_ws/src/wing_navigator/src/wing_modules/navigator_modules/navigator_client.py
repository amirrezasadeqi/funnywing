import rospy
from wing_navigator.srv import ActiveMode, SimpleGoto, ArmTakeoff

class navigator_client:
    def __init__(self, name):
        '''
            {"client_name"}
        '''
        self.name = name

    def active_mode_client(self, req):
        server_name = f"/{self.name}_active_mode"
        rospy.wait_for_service(server_name)
        try:
            active_mode_service = rospy.ServiceProxy(server_name, ActiveMode)
            response = active_mode_service(req)
            return response.accepted
        except rospy.ServiceException as e:
            print("Service Call Failed: %s"%e)

    def simple_goto_client(self, req):
      server_name = f"{self.name}_simple_goto"
      rospy.wait_for_service(server_name)
      try:
        simple_goto_service = rospy.ServiceProxy(server_name, SimpleGoto)
        response = simple_goto_service(req)
        return response.accepted
      except rospy.ServiceException as e:
        print("Service Call Failed: %s"%e)

    def arm_takeoff_client(self, req):
      server_name = f"{self.name}_arm_takeoff"
      rospy.wait_for_service(server_name)
      try:
        arm_takeoff_service = rospy.ServiceProxy(server_name, ArmTakeoff)
        response = arm_takeoff_service(req)
        return response.accepted
      except rospy.ServiceException as e:
        print("Service Call Failed: %s"%e)

