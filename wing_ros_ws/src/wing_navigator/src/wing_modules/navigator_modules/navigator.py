from dronekit import LocationGlobalRelative, connect, VehicleMode, Command, LocationGlobal
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from pymavlink import mavutil
import time
import math
from wing_navigator.srv import PreDefMissionResponse, SimpleGoto, SimpleGotoResponse, ActiveMode, ActiveModeResponse, ArmTakeoff, ArmTakeoffResponse, MissionInOut, MissionInOutResponse
from wing_modules.navigator_modules.TimerAP import TimerAP

#### Test for custom service for mission ####
from wing_navigator.srv import WP_list_save, WP_list_saveResponse, WP_list_upload, WP_list_uploadResponse
from wing_navigator.msg import MissionCommand
#############################################
### Experimental Usage of pickle ############
import pickle
from wing_navigator.srv import TestPickleUpload, TestPickleUploadResponse
#############################################

# Using this function for just Determining of the Lat/Lon and for Alt we would use the Relative Alt as you will see in the following


def get_location_meteres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def demo_launch_mission(aLocation, vehicle, aSize=500, land_incline=0.06):
    """
    Creates a simple mission for demo purpose!
    """
    cmds = vehicle.commands
    print("Clearing Any Existing Commands")
    cmds.clear()
    print("Adding New Commands for Our Demo Mission!")

    # the first point (home point) would define in Absolute Global Frame! The rest will be global with relative altitude!
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
             1, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, aLocation.alt))

    # Add take off command. Actually as I checked the lat and lon of this command has no effect in the result at least for the
    # Fix-Wings. In another words I set them to zero, values based on heading and first waypoint location. In all cases it just
    # follow the altitude and nothing more. But Any way, here I would use a waypoint in the way of the first real waypoint.
    wp_tkoff = get_location_meteres(aLocation, 5, -5)
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, wp_tkoff.lat, wp_tkoff.lon, 10))

    # Adding other waypoints
    wp1 = get_location_meteres(aLocation, aSize, -aSize)
    wp2 = get_location_meteres(aLocation, aSize, aSize)
    wp3 = get_location_meteres(aLocation, -aSize, aSize)
    wp4 = get_location_meteres(aLocation, -aSize, -aSize)
    # Determine Operating altitude based on the landing path lenght:
    land_alt = 2 * math.sqrt(2) * aSize * land_incline
    # take care about the altitude of the landing starting point. I think you should determine it based on the proper inclination of the landing rootself.
    # as I tested in SITL too steep landings after a while cause the plane go over the landing point and the landing would take place with a small pitch angle which takes too mauch time.
    # on the other hand as I tested for 0.18 inclination the plane tries to land exactly on the landing point and there were some oscillations in pitch angle. Also test for 6% inclination
    # and it has accurate and oscillations free land maneuver. I don't know exactly that these oscillations are for bad tune or something else but in future ... .
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp1.lat, wp1.lon, land_alt))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp2.lat, wp2.lon, land_alt))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp3.lat, wp3.lon, land_alt))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp4.lat, wp4.lon, land_alt))

    # Adding Landing waypoint at home location
    # cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, 0))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, wp2.lat, wp2.lon, 0))

    print("Upload New Commands to Vehicle!")
    cmds.upload()


def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    missionlist = []
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray = line.split('\t')
                ln_index = int(linearray[0])
                ln_currentwp = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1 = float(linearray[4])
                ln_param2 = float(linearray[5])
                ln_param3 = float(linearray[6])
                ln_param4 = float(linearray[7])
                ln_param5 = float(linearray[8])
                ln_param6 = float(linearray[9])
                ln_param7 = float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue,
                              ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def upload_mission(aFileName, vehicle):
    """
    Upload a mission from a file.
    """
    # Read mission from file
    missionlist = readmission(aFileName)

    # Clear existing mission from vehicle
    print(' Clear Already Uploaded Mission')
    cmds = vehicle.commands
    cmds.clear()

    print("\nUpload mission from a file: %s" % aFileName)
    # Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()


def download_mission(vehicle):
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist = []
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist


def save_mission(aFileName, vehicle):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)
    # Download mission from vehicle
    missionlist = download_mission(vehicle)
    # Add file-format information
    output = 'QGC WPL 110\n'
    # Add home location as 0th waypoint
    home = vehicle.home_location
    output += "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        0, 1, 0, 16, 0, 0, 0, 0, home.lat, home.lon, home.alt, 1)
    # Add commands
    for cmd in missionlist:
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            cmd.seq, cmd.current, cmd.frame, cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.x, cmd.y, cmd.z, cmd.autocontinue)
        output += commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)


class navigator:
    """
    A Class which contains all the methods for navigation purpose
    list_of_servers_dict : A list of dictionaries like bellow:
        {"server_name": "name", "server_data_type": type, "server_handler_type": "handler_type"}
    Also we can have a handler type map dictionary like bellow:
        {"handler_type": handler_function}
    """

    def __init__(self, agent_name, connection_string, list_of_servers_dict, list_of_pubs_dict, list_of_subs_dict):

        # TODO: delete this
        self.test_counter = 0
        ################

        print(f"Connecting to {agent_name} on: {connection_string}")
        self.vehicle = connect(connection_string, wait_ready=True)
        self.agent_name = agent_name
        # handler function mapping dictionary
        self.handler_mapping = {"simple_goto": self.simple_goto_handler,
                                "active_mode": self.active_mode_handler,
                                "arm_takeoff": self.arm_takeoff_handler,
                                "save_mission": self.save_mission,
                                "upload_mission": self.upload_mission,
                                # these are for test
                                "save_mission_ros": self.save_mission_ros,
                                "upload_mission_ros": self.upload_mission_ros,
                                "upload_predefined_mission": self.upload_predefined_mission,
                                # Handler functions relating to publishers
                                "gps_pub_handler": self.gps_pub_handler
                                ## Experimental pickle ##
                                # "save_mission_pickle": self.save_mission_pickle,
                                # "upload_mission_pickle": self.upload_mission_pickle
                                }
        # Dictionary of servers
        self.dict_servers = {}

        # Dictionary of publishers
        # {publisher_name: {"pulisher_object": publisher_object, "publisher_timer": publisher_timer, "publisher_callback_args": (<Tuple of necessary args)}}
        # the tuple added for the cases that multiple publishers use the same callback function but they need to pass arguments to the callback function which
        # are specific to that specific publisher. TODO: Actually I don't know this way of passing arguments, but the rospy.Timer initializer does not allow to
        # directly apass args, so oneway could be customize rospy.Timer class by inheritance which I will try it in future.
        # TODO: the problem of unknown gps publisher exists yet an solutions can be:
        # 1. Using lamda as the callback function maybe a good choice
        # 2. In callback function we should detect who is the caller
        # so we can publish data with proper publisher object
        self.dict_pubs = {}

        # Dictionary of subscribers
        self.dict_subs = {}

        for server in list_of_servers_dict:
            server_name = server["server_name"]
            server_name = f"/{agent_name}_{server_name}"
            self.dict_servers[server_name] = rospy.Service(
                server_name, server["server_data_type"], self.handler_mapping[server["server_handler_type"]])
            print(f"{server_name} server is at your service!")

        for pub in list_of_pubs_dict:
            pub_name = pub["publisher_name"]
            pub_name = f"/{agent_name}_{pub_name}"
            topic_name = pub["topic_name"]
            topic_name = f"/{agent_name}_{topic_name}"
            # TODO: update the GPS sensor data type to sensor_msgs/NavSatFix message
            self.dict_pubs[pub_name] = {}
            self.dict_pubs[pub_name]["publisher_object"] = rospy.Publisher(
                topic_name, pub["publisher_data_type"], queue_size=pub["queue_size"])

            # Duration[nano seconds]
            self.dict_pubs[pub_name]["publisher_timer"] = TimerAP(rospy.Duration(0, pow(
                10.0, 9.0) / pub["rate"]), self.handler_mapping[pub["pub_handler_type"]], [pub_name])
            # Duration[nano seconds]
            # self.dict_pubs[pub_name]["publisher_timer"] = rospy.Timer(rospy.Duration(
            #     0, pow(10.0, 9.0) / pub["rate"]), self.handler_mapping[pub["pub_handler_type"]])

            #######################################################################################
            #                       Test for using Lambda for passign args to callback
            #######################################################################################
            # self.dict_pubs["pub_name"]["publisher_timer"] = rospy.Timer(rospy.Duration(
            #     0, pow(10.0, 9.0) / pub["rate"]), lambda: self.handle_callback_invokation(pub_name, self.handler_mapping[pub["pub_handler_type"]]))

            # self.dict_pubs[pub_name]["publisher_timer"] = rospy.Timer(rospy.Duration(
            #     0, pow(10.0, 9.0) / pub["rate"]), lambda: self.handler_mapping[pub["pub_handler_type"]](pub_name))

            #######################################################################################
            #       Test Overriding of the Timer class (TimerAP) for passing args to callback
            #######################################################################################

            # Empty tuple at the moment but it may be necessary in future
            self.dict_pubs[pub_name]["publisher_callback_args"] = ()

        for sub in list_of_subs_dict:
            sub_name = sub["subscriber_name"]
            sub_name = f"/{agent_name}_{sub_name}"
            topic_name = sub["topic_name"]
            topic_name = f"/{agent_name}_{topic_name}"
            self.dict_subs[sub_name] = {}
            self.dict_subs[sub_name]["subscriber_object"] = rospy.Subscriber(
                topic_name, sub["subscriber_data_type"], callback.func)

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

    # def handle_callback_invokation(self, pub_name, pub_handler):
    #     return

    def gps_pub_handler(self, arg_list, event=None):
        """
            Read and publish the GPS sensor data on rospy.Timer callbacks
        """
        publisher_object = self.dict_pubs[arg_list[0]]["publisher_object"]
        msg = String()
        self.test_counter += 1
        msg.data = f"{self.test_counter}: Hello World wing publisher!"
        publisher_object.publish(msg)

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

    def save_mission(self, req):
        res = MissionInOutResponse()
        try:
            save_mission(req.filename, self.vehicle)
            res.accepted = True
            return res
        except:
            res.accepted = False
            return res

    def upload_mission(self, req):
        res = MissionInOutResponse()
        try:
            upload_mission(req.filename, self.vehicle)
            res.accepted = True
            return res
        except:
            res.accepted = False
            return res

    def save_mission_ros(self, req):
        res = WP_list_saveResponse()
        res.waypoints = []
        try:
            print(f"Serving the request {req.req_message}")
            mission_list = download_mission(self.vehicle)
            home = self.vehicle.home_location
            res.home_lat = float(home.lat)
            res.home_lon = float(home.lon)
            res.home_alt = float(home.alt)
            for cmd in mission_list:
                wp = MissionCommand()
                wp.ln_0 = int(cmd.target_system)
                wp.ln_1 = int(cmd.target_component)
                wp.ln_2 = int(cmd.seq)
                wp.ln_frame = int(cmd.frame)
                wp.ln_command = int(cmd.command)
                wp.ln_currentwp = int(cmd.current)
                wp.ln_autocontinue = int(cmd.autocontinue)
                wp.ln_param1 = float(cmd.param1)
                wp.ln_param2 = float(cmd.param2)
                wp.ln_param3 = float(cmd.param3)
                wp.ln_param4 = float(cmd.param4)
                wp.ln_param5 = float(cmd.x)
                wp.ln_param6 = float(cmd.y)
                wp.ln_param7 = float(cmd.z)
                res.waypoints.append(wp)

            return res
        except:
            print(
                f"There is some problems in fetching current mission on {self.agent_name}")
            return res

    def upload_mission_ros(self, req):
        res = WP_list_uploadResponse()
        try:
            waypoints = req.waypoints
            mission_list = []
            for wp in waypoints:
                ln_frame = wp.ln_frame
                ln_command = wp.ln_command
                ln_currentwp = wp.ln_currentwp
                ln_autocontinue = wp.ln_autocontinue
                ln_param1 = wp.ln_param1
                ln_param2 = wp.ln_param2
                ln_param3 = wp.ln_param3
                ln_param4 = wp.ln_param4
                ln_param5 = wp.ln_param5
                ln_param6 = wp.ln_param6
                ln_param7 = wp.ln_param7
                cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue,
                              ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                mission_list.append(cmd)

            # Clear existing mission from vehicle
            print('Clear existing mission from vehicle')
            cmds = self.vehicle.commands
            cmds.clear()

            # Add new mission to vehicle
            for command in mission_list:
                cmds.add(command)

            print(' Upload  new mission')
            self.vehicle.commands.upload()

            res.accepted = True
            return res
        except:
            res.accepted = False
            return res

    def save_mission_pickle(self, req):
        res = TestPickleUploadResponse()
        try:
            # do your job
            serialized_mission = req.serialized_mission
            mission_list = pickle.loads(serialized_mission)

            # Clear existing mission from vehicle
            print('Clear existing mission from vehicle')
            cmds = self.vehicle.commands
            cmds.clear()

            # Add new mission to vehicle
            for command in mission_list:
                cmds.add(command)

            print(' Upload  new mission')
            self.vehicle.commands.upload()

            res.accepted = True
            return res
        except:
            res.accepted = False
            return res

    def upload_predefined_mission(self, req):
        # dictionary to map predefined mission names to their mission cunstructors
        mission_constructor_map = {
            "square_mission": self.square_mission_constructor,
            "target_mission": self.target_mission_constructor
        }
        res = PreDefMissionResponse()

        try:
            mission_list = mission_constructor_map[req.mission_type]()

            # Clear existing mission from vehicle
            print('Clear existing mission from vehicle')
            cmds = self.vehicle.commands
            cmds.clear()

            # Add new mission to vehicle
            for command in mission_list:
                cmds.add(command)

            print(' Upload  new mission')
            self.vehicle.commands.upload()

            res.accepted = True
            return res
        except:
            res.accepted = False
            return res

    def square_mission_constructor(self, aSize=500, land_incline=0.06):

        mission_list = []
        aLocation = self.vehicle.location.global_frame
        # the first point (home point) would define in Absolute Global Frame! The rest will be global with relative altitude!
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, aLocation.alt))

        # Add take off command. Actually as I checked the lat and lon of this command has no effect in the result at least for the
        # Fix-Wings. In another words I set them to zero, values based on heading and first waypoint location. In all cases it just
        # follow the altitude and nothing more. But Any way, here I would use a waypoint in the way of the first real waypoint.
        wp_tkoff = get_location_meteres(aLocation, 5, -5)
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, wp_tkoff.lat, wp_tkoff.lon, 10))

        # Adding other waypoints
        wp1 = get_location_meteres(aLocation, aSize, -aSize)
        wp2 = get_location_meteres(aLocation, aSize, aSize)
        wp3 = get_location_meteres(aLocation, -aSize, aSize)
        wp4 = get_location_meteres(aLocation, -aSize, -aSize)
        # Determine Operating altitude based on the landing path lenght:
        land_alt = 2 * math.sqrt(2) * aSize * land_incline
        # take care about the altitude of the landing starting point. I think you should determine it based on the proper inclination of the landing rootself.
        # as I tested in SITL too steep landings after a while cause the plane go over the landing point and the landing would take place with a small pitch angle which takes too mauch time.
        # on the other hand as I tested for 0.18 inclination the plane tries to land exactly on the landing point and there were some oscillations in pitch angle. Also test for 6% inclination
        # and it has accurate and oscillations free land maneuver. I don't know exactly that these oscillations are for bad tune or something else but in future ... .
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp1.lat, wp1.lon, land_alt))
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp2.lat, wp2.lon, land_alt))
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp3.lat, wp3.lon, land_alt))
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp4.lat, wp4.lon, land_alt))

        # Adding Landing waypoint at home location
        # cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, 0))
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, wp2.lat, wp2.lon, 0))

        return mission_list

    def target_mission_constructor(self):
        mission_list = []
        # TODO: here bellow is something to just test. later you should implement a suitable mission for your simulation situation
        aLocation = self.vehicle.location.global_frame
        # Adding other waypoints
        aSize = 500
        wp1 = get_location_meteres(aLocation, aSize, -aSize)
        wp2 = get_location_meteres(aLocation, aSize, aSize)
        wp3 = get_location_meteres(aLocation, -aSize, aSize)
        wp4 = get_location_meteres(aLocation, -aSize, -aSize)

        land_alt = 30

        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp1.lat, wp1.lon, land_alt))
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp2.lat, wp2.lon, land_alt))
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp3.lat, wp3.lon, land_alt))
        mission_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp4.lat, wp4.lon, land_alt))

        return mission_list

    # def fw_gps_publisher(self, event=None):
    #     """
    #         Read and pulisher GPS sensor data on rospy.Timer callbacks
    #     """

    #     # Reading data and substitude it in the msg container
    #     location = self.vehicle.location.global_relative_frame

    #     msg = NavSatFix()
    #     msg.header = self.__get_header__()
    #     Msg.header.frame_id = 'gps'
    #     msg.latitude = float(location.lat)
    #     msg.longitude = float(location.lon)
    #     msg.altitude = float(location.alt)
    #     msg.status.status = NavSatStatus.STATUS_SBAS_FIX
    #     msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
    #     return

    # def __get_header__(self):
    #     """
    #     Returns ROS message header
    #     """
    #     header = Header()
    #     header.stamp = rospy.Time.from_sec(self.timestamp)
    #     return header


class fw_navigator(navigator):
    # TODO: Can't give default values to the constructor of this subclasse and I don't know why!?
    # Please check this out when you have time!
    # def __init__(self, agent_name, connection_string, list_of_servers_dict=[{"server_name": "simple_goto", "server_data_type": SimpleGoto, "server_handler_type": "simple_goto"},
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
