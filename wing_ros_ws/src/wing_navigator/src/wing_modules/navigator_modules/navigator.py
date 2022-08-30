from dronekit import LocationGlobalRelative, connect, VehicleMode
import rospy
import time
from wing_navigator.srv import SimpleGoto, SimpleGotoResponse, ActiveMode, ActiveModeResponse, ArmTakeoff, ArmTakeoffResponse
                               

class navigator:
    """
    A Class which contains all the methods for navigation purpose
    list_of_servers_dict : A list of dictionaries like bellow:
        {"server_name": "name", "server_data_type": type, "server_handler_type": "handler_type"}
    Also we can have a handler type map dictionary like bellow:
        {"handler_type": handler_function}
    """

    def __init__(self, agent_name, connection_string, list_of_servers_dict):
        print(f"Connecting to {agent_name} on: {connection_string}")
        self.vehicle = connect(connection_string, wait_ready=True)
        self.agent_name = agent_name
        # handler function mapping dictionary
        self.handler_mapping = {"simple_goto": self.simple_goto_handler,
                                "active_mode": self.active_mode_handler,
                                "arm_takeoff": self.arm_takeoff_handler}
        # Dictionary of servers
        self.dict_servers = {}

        for server in list_of_servers_dict:
            server_name = server["server_name"]
            server_name = f"/{agent_name}_{server_name}"
            self.dict_servers[server_name] = rospy.Service(server_name, server["server_data_type"], self.handler_mapping[server["server_handler_type"]])
            print(f"{server_name} server is at your service!")


    # def __init__(self, connection_string):
    #     print("Connecting to vehicle on: %s" % connection_string)
    #     self.vehicle = connect(connection_string, wait_ready=True)
    #     self.goto_server = rospy.Service("/simple_goto", SimpleGoto, self.simple_goto_handler)
    #     print("Simple goto is at your service!")
    #     self.active_mode_server = rospy.Service("/active_mode", ActiveMode, self.active_mode_handler)
    #     print("Activation Mode Server is at your service!")
    #     self.arm_takeoff_server = rospy.Service("/arm_takeoff", ArmTakeoff, self.arm_takeoff_handler)
    #     print("Arm and Takeoff Server is at your service!")

    def __del__(self):
        print(f"Close {self.agent_name} Vehicle connection object!")
        self.vehicle.close()

    def arm_takeoff_handler(self, req):
        # resp = ArmTakeoffResponse()
        # try:
        #     self.vehicle_arm()
        #     resp.accepted = True
        #     return resp
        # except:
        #     resp.accepted = False
        #     return resp
        return

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
      # print("Change Mode to AUTO")
      # self.vehicle.mode = VehicleMode("TAKEOFF")

      # self.wait_for_pre_flights()

      # print("Arming Motors!")
      # self.vehicle.arm()
      
      # Must be Override in subclasses
      return
    
    def wait_for_pre_flights(self):
      print("Wait for Pre-flight Checks!")
      while not self.vehicle.is_armable:
        print("Wait for Consistent Sensor Datas and Pre-flight checks!")
        time.sleep(1)

      print("Pre-Flight Checks Passed!")


class fw_navigator(navigator):
    # TODO: Can't give default values to the constructor of this subclasse and I don't know why!?
    # Please check this out when you have time!
    # def __ini__(self, agent_name, connection_string, list_of_servers_dict=[{"server_name": "simple_goto", "server_data_type": SimpleGoto, "server_handler_type": "simple_goto"},
    #                                                                        {"server_name": "active_mode", "server_data_type": ActiveMode, "server_handler_type": "active_mode"},
    #                                                                        {"server_name": "arm_takeoff", "server_data_type": ArmTakeoff, "server_handler_type": "arm_takeoff"}]):
    #     # Using navigator class constructor
    #     super().__init__(agent_name, connection_string, list_of_servers_dict)

    # TODO: I think this does not need to be overriden but there is not time to check it, so check it later!
    def arm_takeoff_handler(self, req):
        resp = ArmTakeoffResponse()
        try:
            self.vehicle_arm()
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

      super().wait_for_pre_flights()

      print("Arming Motors!")
      self.vehicle.arm()

class copter_navigator(navigator):
    # TODO: Can't give default values to the constructor of this subclasse and I don't know why!?
    # Please check this out when you have time!
    # def __ini__(self, agent_name, connection_string, list_of_servers_dict = [{"server_name": "simple_goto", "server_data_type": SimpleGoto, "server_handler_type": "simple_goto"},
    #                                                                          {"server_name": "active_mode", "server_data_type": ActiveMode, "server_handler_type": "active_mode"},
    #                                                                          {"server_name": "arm_takeoff", "server_data_type": ArmTakeoff, "server_handler_type": "arm_takeoff"}]):
    #     # Using navigator class constructor
    #     super().__init__(agent_name, connection_string, list_of_servers_dict)

    # TODO: I think this does not need to be overriden but there is not time to check it, so check it later!
    def arm_takeoff_handler(self, req):
        resp = ArmTakeoffResponse()
        try:
            self.vehicle_arm()
            resp.accepted = True
            return resp
        except:
            resp.accepted = False
            return resp

    def vehicle_arm(self):
      """
        Safely Arm the vehicle!
      """
      print("Change Mode to GUIDED")
      self.vehicle.mode = VehicleMode("GUIDED")

      super().wait_for_pre_flights()

      print("Arming Motors!")
      self.vehicle.arm()

      print("Start Takeoff process")
      # takeoff to a default altitude 20 meters
      self.vehicle.simple_takeoff(20)

