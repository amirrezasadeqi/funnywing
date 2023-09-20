#!/usr/bin/env python

import subprocess as sp
import sys
import threading
from os import symlink, makedirs
from os.path import expanduser, exists

import msgpack
import numpy as np
import pymap3d as pm
import rospy
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from dronekit import Command, LocationGlobal
from pymavlink import mavutil
from rospkg import RosPack as rospack
from std_msgs.msg import UInt8MultiArray
from wing_navigator.msg import GLOBAL_POSITION_INT, MissionCommand
from wing_navigator.srv import PreDefMissionRequest, SimpleGotoRequest, ActiveModeRequest, ArmTakeoffRequest, \
    MissionInOutRequest
#### Test for custom service for mission ####
from wing_navigator.srv import WP_list_saveRequest, WP_list_uploadRequest

from wing_modules.navigator_modules.navigation_commands import navigation_commands as nav_com
from wing_modules.navigator_modules.navigator_client import navigator_client


#############################################
### TODO: Experimental Usage of pickle ############
#############################################

# Threading Experimental
# Note that the errors produced in the virtual texts are not really important
# and this is a bug in PyQt5
#############################################################################


class client_worker(QObject):
    finished = pyqtSignal()

    def run(self):
        """
            Do the works that must be done by clicking here
        """
        req = ArmTakeoffRequest()
        tg_client = navigator_client("target")
        print(f"{tg_client.name} Requests for Arm and Takeoff vehicle")
        # print("Request Result: %s"%arm_takeoff_client(req))
        print("Request Result: %s" % tg_client.arm_takeoff_client(req))
        self.finished.emit()


def real2virt_target_pos_converter(tg_global_pos, tg_global_lon, tg_global_alt, fw_lat, fw_lon, fw_alt, offset):
    lat0, lon0, alt0 = 35.41323864, 51.15932969, 1007
    tg_local_pos = pm.geodetic2enu(
        tg_global_pos, tg_global_lon, tg_global_alt, lat0, lon0, alt0)
    fw_local_pos = pm.geodetic2enu(
        fw_lat, fw_lon, fw_alt, lat0, lon0, alt0)
    diff_vec = np.array(tg_local_pos) - np.array(fw_local_pos)
    diff_unit_vec = diff_vec/np.linalg.norm(diff_vec)
    virt_tg_local_pos = tg_local_pos + offset * diff_unit_vec
    virt_tg_global_pos = pm.enu2geodetic(
        virt_tg_local_pos[0], virt_tg_local_pos[1], virt_tg_local_pos[2], lat0, lon0, alt0)
    virt_tg_lat = virt_tg_global_pos[0]
    virt_tg_lon = virt_tg_global_pos[1]
    virt_tg_alt = virt_tg_global_pos[2]
    return virt_tg_lat, virt_tg_lon, virt_tg_alt


class run_test_worker(QObject):
    finished = pyqtSignal()

    def __init__(self, test_type="simple_tracker"):
        super(run_test_worker, self).__init__()
        self._test_type = test_type
        self._test_handler = {
            "simple_tracker": self._simple_tracker_callback,
            "pn_tracker": self._pn_tracker_callback,
            "unknown": self._unknown
        }

    def _simple_tracker_callback(self, msg, args):
        publisher = args[0]
        # real target gps location
        tg_lat = msg.gps_data.latitude
        tg_lon = msg.gps_data.longitude
        tg_alt = msg.gps_data.altitude
        fw_lat, fw_lon, fw_alt = last_fw_global_pos[0], last_fw_global_pos[1], last_fw_global_pos[2]
        virt_tg_lat, virt_tg_lon, virt_tg_alt = real2virt_target_pos_converter(
            tg_lat, tg_lon, tg_alt, fw_lat, fw_lon, fw_alt, 120)
        cmd = nav_com.simple_goto(virt_tg_lat, virt_tg_lon, virt_tg_alt)
        packed_cmd = msgpack.packb(cmd)
        cmd_msg = UInt8MultiArray()
        cmd_msg.data = packed_cmd
        publisher.publish(cmd_msg)

    def _pn_tracker_callback(self, msg, args):
        rospy.loginfo("This tracking algorithm is Not implemented yet!")

    def _unknown(self, msg, args):
        rospy.loginfo("Unsupported type of tracker!")

    def run(self):
        publisher = rospy.Publisher(
            "/wing_nav_cmds_gcs", UInt8MultiArray, queue_size=1)
        tg_gps_subscriber = rospy.Subscriber(
            "/target_gps_topic", GLOBAL_POSITION_INT, self._test_handler[self._test_type], (publisher, ))
        rospy.spin()
        self.finished.emit()


class subscription_worker(QObject):
    finished = pyqtSignal()

    def __init__(self):
        super(subscription_worker, self).__init__()
        self.subs_handler_mapping = {"gps_sub_handler": self.gps_sub_handler,
                                     "cmd_resp_handler": self.cmd_resp_handler}
        self.list_of_subs_dict = [{"subscriber_name": "gps_subscriber", "topic_name": "gps_topic_gcs",
                                   "subscriber_data_type": GLOBAL_POSITION_INT, "sub_handler_type": "gps_sub_handler"},
                                  {"subscriber_name": "cmd_resp_subscriber", "topic_name": "cmd_resp_topic_gcs",
                                   "subscriber_data_type": UInt8MultiArray, "sub_handler_type": "cmd_resp_handler"}]

        # Initialize the container for GUI app subscriber objects.
        self.dict_subs = {}

        for sub in self.list_of_subs_dict:
            sub_name = sub["subscriber_name"]
            sub_name = f"/wing_{sub_name}"
            topic_name = sub["topic_name"]
            topic_name = f"/wing_{topic_name}"
            self.dict_subs[sub_name] = {}
            self.dict_subs[sub_name]["subscriber_object"] = rospy.Subscriber(
                topic_name, sub["subscriber_data_type"], self.subs_handler_mapping[sub["sub_handler_type"]])
            self.dict_subs[sub_name]["subscriber_callback_args"] = ()

    def run(self):
        """
            Do the subscription to data by spinning via rospy.spin(). after spin breaks or
            terminates the looping, it emits a signal for finishing the object(I think) and
            thread.
        """
        # Note: I have some doubts about the way application closing terminates rospy.spin()
        # But I think it will be ok, if any issues in terminating or any wierd behavior
        # , take a look at this.

        rospy.spin()

        self.finished.emit()

    def gps_sub_handler(self, msg):
        """
            Reading GPS data from the topic which is published by the RF module
            and visualize it in the GUI app or use it for other works.
        """
        # Just printing out the subscribed data into terminal to test the code correctness.
        # rospy.loginfo(
        #     f"\nSome data to check embedding subscriber into GUI app:\nlatitude: {msg.gps_data.latitude}\nlongitude: {msg.gps_data.longitude}\n altitude: {msg.gps_data.altitude}\n")
        last_fw_global_pos = [msg.gps_data.latitude,
                              msg.gps_data.longitude, msg.gps_data.altitude]

    def cmd_resp_handler(self, msg):
        '''
            Reading Command responses from the topic which is published by the RF module
            and gives the response the requester.
        '''
        mav_msg = msgpack.unpackb(msg.data)
        # mav_msg is checked in the publisher side, so you can directly use get_type() or to_dict
        if mav_msg.get_type() == "COMMAND_RESPONSE":
            mav_msg = mav_msg.to_dict()
            rospy.loginfo(
                f"Response to Request number {mav_msg['request_number']}: {mav_msg['result']}")
        else:
            rospy.loginfo(
                f"The command type {mav_msg.get_type()} is not supported yet!")

#############################################################################


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


def uilink_if_needed():

    # Address of symlink to ui file
    uilink_path = expanduser("~/.ros/demo_app/demo_app.ui")

    # Address of the symlink directory
    uilink_dir = expanduser("~/.ros/demo_app")

    # Address of the original ui file.
    uifile_path = rospack().get_path("wing_navigator") + "/scripts/demo_app.ui"

    if not exists(uilink_path):
        if not exists(uilink_dir):
            makedirs(uilink_dir)
        symlink(uifile_path, uilink_path)

    return uilink_path


def abbreviate_location(long_loc):
    if long_loc == "Imam Khomeini Airport":
        return "IKA"
    elif long_loc == "Mehrabad Airport":
        return "MA"
    elif long_loc == "SWITehran":
        return "SWIT"
    else:
        print("Please Enter a Valid Location!")

# def active_mode_client(req):
#     rospy.wait_for_service("/active_mode")
#     try:
#         active_mode_service = rospy.ServiceProxy("/active_mode", ActiveMode)
#         response = active_mode_service(req)
#         return response.accepted
#     except rospy.ServiceException as e:
#         print("Service Call Failed: %s"%e)

# def simple_goto_client(req):
#   rospy.wait_for_service("/simple_goto")
#   try:
#     simple_goto_service = rospy.ServiceProxy("/simple_goto", SimpleGoto)
#     response = simple_goto_service(req)
#     return response.accepted
#   except rospy.ServiceException as e:
#     print("Service Call Failed: %s"%e)

# def arm_takeoff_client(req):
#   rospy.wait_for_service("/arm_takeoff")
#   try:
#     arm_takeoff_service = rospy.ServiceProxy("/arm_takeoff", ArmTakeoff)
#     response = arm_takeoff_service(req)
#     return response.accepted
#   except rospy.ServiceException as e:
#     print("Service Call Failed: %s"%e)


class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uilink_path = uilink_if_needed()
        uic.loadUi(uilink_path, self)

        self.mode_mapping = {
            "MANUAL": 1,
            "STABILIZE": 2,
            "AUTO": 3,
            "GUIDED": 4,
            "TAKEOFF": 5,
            "LOITER": 6,
            "RTL": 7,
            "FBWB": 8,
            "CIRCLE": 9,
            "FBWA": 10
        }
        ################################################################
        # Adding Subscription to sensor data for visualization in GUI.
        # this section is under test and developement.
        # Note that this code will be ran on GCS computer.
        ################################################################

        # TODO: I think we need a seperate thread for sensor data subscription. and we need
        # to initialize the thread in the application constructor before(before/after may not
        # have effect) the show() function call for starting subscription to data even before
        # seeing the GUI on the monitor.

        # setup and start the seperated thread for subscriptions.
        self.subs_thread = QThread()
        # This also creates the subscribers and encapsulates the last gps data of funnywing
        self.subs_worker = subscription_worker()
        self.subs_worker.moveToThread(self.subs_thread)
        # subscription by spin() which starts by thread start
        self.subs_thread.started.connect(self.subs_worker.run)
        self.subs_thread.finished.connect(self.subs_thread.quit)
        self.subs_worker.finished.connect(
            self.subs_worker.deleteLater)
        self.subs_thread.finished.connect(self.subs_thread.deleteLater)
        self.subs_thread.start()

        # self.subs_handler_mapping = {"gps_sub_handler": self.gps_sub_handler}
        # self.list_of_subs_dict = [{"subscriber_name": "gps_subscriber", "topic_name": "gps_topic",
        #                            "subscriber_data_type": NavSatFix, "sub_handler_type": "gps_sub_handler"}]

        # Initialize the container for GUI app subscriber objects.
        # self.dict_subs = {}

        # for sub in self.list_of_subs_dict:
        #     sub_name = sub["subscriber_name"]
        #     sub_name = f"/wing_{sub_name}"
        #     topic_name = sub["topic_name"]
        #     topic_name = f"/wing_{topic_name}"
        #     self.dict_subs[sub_name] = {}
        #     self.dict_subs[sub_name]["subscriber_object"] = rospy.Subscriber(
        #         topic_name, sub["subscriber_data_type"], self.subs_handler_mapping[sub["sub_handler_type"]])
        #     self.dict_subs[sub_name]["subscriber_callback_args"] = ()
        ################################################################

        ################################################################
        # Adding publisher for publishing serialized commands to commands
        # topic for using in RF communication node.
        # this section is under test and developement.
        # Note that this code will be ran on GCS computer.
        ################################################################

        self.list_of_pubs_dict = [
            {"publisher_name": "navigation_commands_publisher", "topic_name": "nav_cmds_gcs",
                "publisher_data_type": UInt8MultiArray, "queue_size": 1}
        ]
        self.dict_pubs = {}
        for pub in self.list_of_pubs_dict:
            pub_name = pub["publisher_name"]
            pub_name = f"/wing_{pub_name}"
            topic_name = pub["topic_name"]
            topic_name = f"/wing_{topic_name}"
            self.dict_pubs[pub_name] = {}
            self.dict_pubs[pub_name]["publisher_object"] = rospy.Publisher(
                topic_name, pub["publisher_data_type"], queue_size=pub["queue_size"])

        ################################################################

        # fixed wing client objects
        self.fw_client = navigator_client("wing")

        # Adding A mission list to save custom waypoints
        self.custom_mission_list = []

        # Simulation Tab objects
        self.gz_world_address = self.findChild(
            QtWidgets.QLineEdit, 'lineEdit_4')
        self.sim_map_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox')
        self.sim_console_chbox = self.findChild(
            QtWidgets.QCheckBox, 'checkBox_2')
        self.sim_multi_vehicle_chbox = self.findChild(
            QtWidgets.QCheckBox, 'checkBox_3')
        self.sim_multi_vehicle_chbox.stateChanged.connect(
            self.sim_multi_vehicle_chbox_toggle_handler)
        self.sim_osd_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox_4')
        self.sim_loc_combo = self.findChild(QtWidgets.QComboBox, 'comboBox')
        self.gzworld_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_3')
        self.gzworld_button.clicked.connect(self.get_gazebo_world_file)
        self.sim_button = self.findChild(QtWidgets.QPushButton, 'pushButton_2')
        self.sim_button.clicked.connect(self.run_simulation)

        # Navigation Center Tab objects
        # Fixed-Wing Navigation Dashboard
        # Mode and Arm/Takeoff ...
        # self.fw_mode_name_text_in = self.findChild(QtWidgets.QLineEdit, 'lineEdit_9')
        self.fw_mode_name_combo_box = self.findChild(
            QtWidgets.QComboBox, 'comboBox_2')
        self.fw_active_mode_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_11')
        self.fw_active_mode_button.clicked.connect(self.fw_active_mode)
        self.fw_arm_takeoff_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_12')
        self.fw_arm_takeoff_button.clicked.connect(self.fw_arm_takeoff)
        self.fw_return_home_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_13')
        self.fw_return_home_button.clicked.connect(self.fw_return_home)

        # Goto Service
        self.fw_goto_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton')
        self.fw_goto_button.clicked.connect(self.fw_gotoButtonPressed)

        self.fw_goto_lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit')
        self.fw_goto_lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_2')
        self.fw_goto_alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_3')

        # Target Navigation Dashboard
        # Mode and Arm/Takeoff ...
        self.tg_mode_name_text_in = self.findChild(
            QtWidgets.QLineEdit, 'lineEdit_17')
        self.tg_mode_name_combo_box = self.findChild(
            QtWidgets.QComboBox, 'comboBox_4')
        self.tg_active_mode_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_21')
        self.tg_active_mode_button.clicked.connect(self.tg_active_mode)
        self.tg_arm_takeoff_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_19')
        self.tg_arm_takeoff_button.clicked.connect(self.tg_arm_takeoff)
        self.tg_return_home_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_20')
        self.tg_return_home_button.clicked.connect(self.tg_return_home)

        # Goto Service
        self.tg_goto_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_18')
        self.tg_goto_button.clicked.connect(self.tg_gotoButtonPressed)

        self.tg_goto_lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit_15')
        self.tg_goto_lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_14')
        self.tg_goto_alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_16')

        # Mission Management Tab objects
        # Import/Export and Pre-Defined Missions
        # Mission File
        self.fw_mission_file_address = self.findChild(
            QtWidgets.QLineEdit, 'lineEdit_8')
        self.fw_mission_file_browse_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_6')
        self.fw_mission_file_browse_button.clicked.connect(
            self.get_fw_mission_file)
        self.fw_save_current_mission_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_7')
        # self.fw_save_current_mission_button.clicked.connect(self.fw_save_current_mission)
        self.fw_save_current_mission_button.clicked.connect(
            self.fw_save_current_mission_ros)
        self.fw_upload_mission_file_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_8')
        # self.fw_upload_mission_file_button.clicked.connect(self.fw_upload_mission_file)
        self.fw_upload_mission_file_button.clicked.connect(
            self.fw_upload_mission_file_ros)
        ###### TODO: Experimental upload using pickle ########################
        # self.fw_upload_mission_file_button.clicked.connect(self.fw_upload_mission_file_pickle)
        ################################################################

        # Pre-Defined Missions
        self.fw_square_mission_radio_button = self.findChild(
            QtWidgets.QRadioButton, 'radioButton')
        self.tg_mission_radio_button = self.findChild(
            QtWidgets.QRadioButton, 'radioButton_2')
        if not self.sim_multi_vehicle_chbox.isChecked():
            self.tg_mission_radio_button.setEnabled(False)
        self.upload_predefined_mission_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_9')
        self.upload_predefined_mission_button.clicked.connect(
            self.upload_predefined_mission)

        # Custom Missions
        self.fw_wp_lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit_5')
        self.fw_wp_lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_6')
        self.fw_wp_alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_7')
        self.fw_wp_table = self.findChild(
            QtWidgets.QTableWidget, 'tableWidget')
        # Table will fit the screen horizontally
        self.fw_wp_table.horizontalHeader().setStretchLastSection(True)
        self.fw_wp_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self.fw_wp_table.verticalHeader().setStretchLastSection(True)
        self.fw_wp_table.verticalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)

        self.fw_clear_wp_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_4')
        self.fw_clear_wp_button.clicked.connect(self.fw_clear_wp)
        self.fw_add_wp_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_5')
        self.fw_add_wp_button.clicked.connect(self.fw_add_wp)
        self.fw_upload_custom_mission_button = self.findChild(
            QtWidgets.QPushButton, 'pushButton_10')
        self.fw_upload_custom_mission_button.clicked.connect(
            self.fw_upload_custom_mission)

        ##############################################################
        #              RF Communication Tab
        #              Section Under Test and developement
        ##############################################################
        self.rf_com_modename_combobox = self.findChild(
            QtWidgets.QComboBox, 'rf_com_mode_combobox')
        self.rf_com_activemode_pushbutton = self.findChild(
            QtWidgets.QPushButton, 'rf_com_activemode_pushbutton')
        self.rf_com_activemode_pushbutton.clicked.connect(
            self.rf_com_activemode_pushbutton_func)
        self.rf_com_returnhome_pushbutton = self.findChild(
            QtWidgets.QPushButton, "rf_com_returnhome_pushbutton")
        self.rf_com_returnhome_pushbutton.clicked.connect(
            self.rf_com_returnhome_pushbutton_func)
        self.rf_com_armtakeoff_pushbutton = self.findChild(
            QtWidgets.QPushButton, 'rf_com_armtakeoff_pushbutton')
        self.rf_com_armtakeoff_pushbutton.clicked.connect(
            self.rf_com_armtakeoff_pushbutton_func)

        ##############################################################
        self.rf_com_simple_goto_lat = self.findChild(
            QtWidgets.QLineEdit, "rf_com_simple_goto_lat")
        self.rf_com_simple_goto_lon = self.findChild(
            QtWidgets.QLineEdit, "rf_com_simple_goto_lon")
        self.rf_com_simple_goto_alt = self.findChild(
            QtWidgets.QLineEdit, "rf_com_simple_goto_alt")
        self.rf_com_simple_goto_push_button = self.findChild(
            QtWidgets.QPushButton, "rf_com_simple_goto_push_button")
        self.rf_com_simple_goto_push_button.clicked.connect(
            self.rf_com_simple_goto)
        ##############################################################
        self.simple_tracker_radiobutton = self.findChild(
            QtWidgets.QRadioButton, "simple_tracker_radiobutton")
        self.pn_tracker_radiobutton = self.findChild(
            QtWidgets.QRadioButton, "pn_tracker_radiobutton")
        self.run_test_pushbutton = self.findChild(
            QtWidgets.QPushButton, "run_test_pushbutton")
        self.run_test_pushbutton.clicked.connect(self.run_test_pushbutton_func)
        self.stop_test_pushbutton = self.findChild(
            QtWidgets.QPushButton, "stop_test_pushbutton")
        self.stop_test_pushbutton.clicked.connect(
            self.stop_test_pushbutton_func)
        ##############################################################

        self.show()

    ##############################################################
    #              Section Under Test and developement
    ##############################################################

    def rf_com_activemode_pushbutton_func(self):
        """
            Function for sending mode selection command over RF com
            Note: This function sends the commands but does not receive
            the response.
        """
        flight_mode = self.rf_com_modename_combobox.currentText()
        flight_mode_number = self.mode_mapping[flight_mode]
        active_mode_command = nav_com.active_mode(
            flight_mode, flight_mode_number)
        packed_active_mode_command = msgpack.packb(active_mode_command)
        publisher_object = self.dict_pubs["/wing_navigation_commands_publisher"]["publisher_object"]
        msg = UInt8MultiArray()
        msg.data = packed_active_mode_command
        publisher_object.publish(msg)
        # No sleep. Just one time publish, I mean publishing one message per click.
        # TODO: add the ability to get the response of the command

    def rf_com_returnhome_pushbutton_func(self):
        """
            Function for sending return to home command over RF com
        """
        flight_mode = "RTL"
        flight_mode_number = self.mode_mapping[flight_mode]
        return_home_command = nav_com.active_mode(
            flight_mode, flight_mode_number)
        packed_return_home_command = msgpack.packb(return_home_command)
        publisher_object = self.dict_pubs["/wing_navigation_commands_publisher"]["publisher_object"]
        msg = UInt8MultiArray()
        msg.data = packed_return_home_command
        publisher_object.publish(msg)
        # No sleep. Just one time publish, I mean publishing one message per click.

    def rf_com_armtakeoff_pushbutton_func(self):
        """
            Function for sending arm takeoff command over RF com
        """
        arm_takeoff_command = nav_com.arm_takeoff()
        packed_arm_takeoff_command = msgpack.packb(arm_takeoff_command)
        publisher_object = self.dict_pubs["/wing_navigation_commands_publisher"]["publisher_object"]
        msg = UInt8MultiArray()
        msg.data = packed_arm_takeoff_command
        publisher_object.publish(msg)
        # No sleep. Just one time publish, I mean publishing one message per click.

    def rf_com_simple_goto(self):
        lat = float(self.rf_com_simple_goto_lat.text())
        lon = float(self.rf_com_simple_goto_lon.text())
        alt = float(self.rf_com_simple_goto_alt.text())
        simple_goto_cmd = nav_com.simple_goto(lat, lon, alt)
        packed_simple_goto_cmd = msgpack.packb(simple_goto_cmd)
        publisher_object = self.dict_pubs["/wing_navigation_commands_publisher"]["publisher_object"]
        msg = UInt8MultiArray()
        msg.data = packed_simple_goto_cmd
        publisher_object.publish(msg)

    def _simple_tracker_callback(self, msg, args):
        publisher = args[0]
        # real target gps location
        tg_lat = msg.gps_data.latitude
        tg_lon = msg.gps_data.longitude
        tg_alt = msg.gps_data.altitude
        fw_lat, fw_lon, fw_alt = last_fw_global_pos[0], last_fw_global_pos[1], last_fw_global_pos[2]
        virt_tg_lat, virt_tg_lon, virt_tg_alt = real2virt_target_pos_converter(
            tg_lat, tg_lon, tg_alt, fw_lat, fw_lon, fw_alt, 120)
        cmd = nav_com.simple_goto(virt_tg_lat, virt_tg_lon, virt_tg_alt)
        packed_cmd = msgpack.packb(cmd)
        cmd_msg = UInt8MultiArray()
        cmd_msg.data = packed_cmd
        publisher.publish(cmd_msg)

    def _pn_tracker_callback(self, msg, args):
        rospy.loginfo("This tracking algorithm is Not implemented yet!")

    def _unknown(self, msg, args):
        rospy.loginfo("Unsupported type of tracker!")

    def run_test_pushbutton_func(self):

        self.test_event = threading.Event()
        self.run_test_thread = threading.Thread(
            target=self._run_test_worker, args=(self.test_event, ))
        self.run_test_thread.start()
        # self.run_test_thread = QThread()
        # self.run_test_worker = run_test_worker(test_type)
        # self.run_test_worker.moveToThread(self.run_test_thread)
        # self.run_test_thread.started.connect(self.run_test_worker.run)
        # self.run_test_thread.finished.connect(self.run_test_thread.quit)
        # self.run_test_worker.finished.connect(
        #     self.run_test_worker.deleteLater)
        # self.run_test_thread.finished.connect(self.run_test_thread.deleteLater)
        # self.run_test_thread.start()

    def _run_test_worker(self):
        self._test_handler = {
            "simple_tracker": self._simple_tracker_callback,
            "pn_tracker": self._pn_tracker_callback,
            "unknown": self._unknown
        }

        if self.simple_tracker_radiobutton.isChecked():
            test_type = "simple_tracker"
        elif self.pn_tracker_radiobutton.isChecked():
            test_type = "pn_tracker"
        else:
            test_type = "unknown"

        self.publisher = rospy.Publisher(
            "/wing_nav_cmds_gcs", UInt8MultiArray, queue_size=1)
        self.tg_gps_subscriber = rospy.Subscriber(
            "/target_gps_topic", GLOBAL_POSITION_INT, self._test_handler[test_type], (self.publisher, ))
        # rospy.spin()

    def stop_test_pushbutton_func(self):
        # if self.run_test_thread and self.run_test_thread.isRunning():
        # self.run_test_thread.quit()
        # self.run_test_thread.wait()
        self.run_test_thread.join()
        ##############################################################

    def get_gazebo_world_file(self):
        worldfile, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, 'Single File', rospack().get_path("wing_navigator") + "/worlds", '*.world')
        if worldfile:
            self.gz_world_address.setText(worldfile)

    def run_simulation(self):
        args = []
        args.append(rospack().get_path("wing_navigator") + "/scripts/run_simulation.bash")

        if self.sim_map_chbox.isChecked():
            args.append("-m")
        if self.sim_console_chbox.isChecked():
            args.append("-c")
        if self.sim_osd_chbox.isChecked():
            args.append("-o")
        if self.sim_multi_vehicle_chbox.isChecked():
            args.append("-s")

        # Location of SITL simulation
        args.append(abbreviate_location(self.sim_loc_combo.currentText()))
        # Gazebo world file
        args.append(self.gz_world_address.text())
        sp.check_call(args)

    def fw_active_mode(self):
        req = ActiveModeRequest(self.fw_mode_name_combo_box.currentText())

        # req.mode = bytes(self.fw_mode_name_combo_box.currentText().encode('utf-8'))
        print(
            f"Requesting to for Mode Activation service for Activating the {req.mode} Flight mode.")
        # print("Request Result: %s"%active_mode_client(req))
        print("Request Result: %s" % self.fw_client.active_mode_client(req))

    def fw_arm_takeoff(self):
        req = ArmTakeoffRequest()
        print(f"{self.fw_client} Requests for Arm and Takeoff vehicle")
        # print("Request Result: %s"%arm_takeoff_client(req))
        print("Request Result: %s" % self.fw_client.arm_takeoff_client(req))

    def fw_return_home(self):
        req = ActiveModeRequest("RTL")
        # req.mode = bytes(self.fw_mode_name_combo_box.currentText().encode('utf-8'))
        print(f"{self.fw_client} Requests for going back to home")
        # print("Request Result: %s"%active_mode_client(req))
        print("Request Result: %s" % self.fw_client.active_mode_client(req))

    def fw_gotoButtonPressed(self):
        # sp.check_call(["./demo_app.bash", self.lat.text(), self.lon.text(), self.alt.text()])
        req = SimpleGotoRequest()
        req.lat = float(self.fw_goto_lat.text())
        req.lon = float(self.fw_goto_lon.text())
        req.alt = float(self.fw_goto_alt.text())
        print(f"{self.fw_client} is Requesting for Simple goto service to point (lat:%s, lon:%s, alt:%s)" % (
            req.lat, req.lon, req.alt))
        print("Request Result: %s" % self.fw_client.simple_goto_client(req))

    # def sim_multi_vehicle_chbox_toggle_handler(self):
    #     if self.sim_multi_vehicle_chbox.isChecked():
    #         if not self.tg_client:
    #             self.tg_client = navigator_client("target")
    #     else:
    #         if self.tg_client:
    #             del self.tg_client

    def sim_multi_vehicle_chbox_toggle_handler(self):
        if self.sim_multi_vehicle_chbox.isChecked():
            self.tg_mission_radio_button.setEnabled(True)
        else:
            self.tg_mission_radio_button.setEnabled(False)

    def tg_active_mode(self):
        req = ActiveModeRequest(self.tg_mode_name_combo_box.currentText())
        tg_client = navigator_client("target")
        # req.mode = bytes(self.fw_mode_name_combo_box.currentText().encode('utf-8'))
        print(
            f"Requesting to for Mode Activation service for Activating the {req.mode} Flight mode.")
        # print("Request Result: %s"%active_mode_client(req))
        print("Request Result: %s" % tg_client.active_mode_client(req))

    # def tg_arm_takeoff(self):
    #     req = ArmTakeoffRequest()
    #     tg_client = navigator_client("target")
    #     print(f"{tg_client.name} Requests for Arm and Takeoff vehicle")
    #     # print("Request Result: %s"%arm_takeoff_client(req))
    #     print("Request Result: %s"%tg_client.arm_takeoff_client(req))

    # Using QThread to prevent GUI freezes
    def tg_arm_takeoff(self):
        self.tg_arm_takeoff_thread = QThread()
        self.tg_client_worker = client_worker()
        self.tg_client_worker.moveToThread(self.tg_arm_takeoff_thread)
        self.tg_arm_takeoff_thread.started.connect(self.tg_client_worker.run)
        self.tg_arm_takeoff_thread.finished.connect(
            self.tg_arm_takeoff_thread.quit)
        self.tg_client_worker.finished.connect(
            self.tg_client_worker.deleteLater)
        self.tg_arm_takeoff_thread.finished.connect(
            self.tg_arm_takeoff_thread.deleteLater)
        self.tg_arm_takeoff_thread.start()

        # TODO: This reset does not work currently because dronekit takeoff does not compelete
        # and I think the thread does not finish. Solve it later! for now just reactivate the
        # button on other events.
        # disable the button arm and takeoff button to prevent wrong click events
        # self.tg_arm_takeoff_button.setEnabled(False)
        # self.tg_arm_takeoff_thread.finished.connect(
        #         # activate the button after thread job is done.
        #         lambda : self.tg_arm_takeoff_button.setEnabled(True)
        #         )

    def tg_return_home(self):
        req = ActiveModeRequest("RTL")
        tg_client = navigator_client("target")
        # req.mode = bytes(self.fw_mode_name_combo_box.currentText().encode('utf-8'))
        print(f"{tg_client.name} Requests for going back to home")
        # print("Request Result: %s"%active_mode_client(req))
        print("Request Result: %s" % tg_client.active_mode_client(req))

    def tg_gotoButtonPressed(self):
        # sp.check_call(["./demo_app.bash", self.lat.text(), self.lon.text(), self.alt.text()])
        req = SimpleGotoRequest()
        tg_client = navigator_client("target")
        req.lat = float(self.tg_goto_lat.text())
        req.lon = float(self.tg_goto_lon.text())
        req.alt = float(self.tg_goto_alt.text())
        print("Target Requesting to for Simple goto service to point (lat:%s, lon:%s, alt:%s)" % (
            req.lat, req.lon, req.alt))
        print("Target Request Result: %s" % tg_client.simple_goto_client(req))

    def get_fw_mission_file(self):
        fw_mission_file, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, 'Single File', expanduser("~"), '*.txt')
        if fw_mission_file:
            self.fw_mission_file_address.setText(fw_mission_file)

    def fw_save_current_mission(self):
        req = MissionInOutRequest()
        fw_client = navigator_client("wing")
        req.filename = self.fw_mission_file_address.text()
        print("Requesting to for save current mission service")
        print("Request Result: %s" % fw_client.save_mission_client(req))

    def fw_save_current_mission_ros(self):
        req = WP_list_saveRequest()
        fw_client = navigator_client("wing")
        req.req_message = f"{fw_client.name} requests to save the current vehicle mission to local file: {self.fw_mission_file_address.text()}"
        # Fetching mission from vehicle with custom service format
        response = fw_client.save_mission_ros_client(req)
        mission_list_resp = response.waypoints
        # print("Request Result: %s"%tg_client.save_mission_client(req))
        ####################################################################
        # Add file-format information
        output = 'QGC WPL 110\n'
        # Add home location as 0th waypoint
        home_lat = response.home_lat
        home_lon = response.home_lon
        home_alt = response.home_alt
        output += "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            0, 1, 0, 16, 0, 0, 0, 0, home_lat, home_lon, home_alt, 1)
        # Add commands
        for cmd in mission_list_resp:
            # commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
            commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
                cmd.ln_2, cmd.ln_currentwp, cmd.ln_frame, cmd.ln_command, cmd.ln_param1, cmd.ln_param2, cmd.ln_param3, cmd.ln_param4, cmd.ln_param5, cmd.ln_param6, cmd.ln_param7, cmd.ln_autocontinue)
            output += commandline

        aFileName = self.fw_mission_file_address.text()
        with open(aFileName, 'w') as file_:
            print(" Write mission to file")
            file_.write(output)
        #############################################################################

    def fw_upload_mission_file(self):
        mission_file = self.fw_mission_file_address.text()
        req = MissionInOutRequest()
        tg_client = navigator_client("wing")
        req.filename = mission_file
        print("Requesting to for upload mission file service")
        print("Request Result: %s" % tg_client.upload_mission_client(req))

    def fw_upload_mission_file_ros(self):
        mission_file = self.fw_mission_file_address.text()
        mission_list = readmission(mission_file)
        req = WP_list_uploadRequest()
        req.waypoints = []
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
            req.waypoints.append(wp)

        fw_client = navigator_client("wing")
        print(
            f"{fw_client.name} Requesting for over ros network upload mission file service!")
        print("Request Result: %s" % fw_client.upload_mission_ros_client(req))

    def upload_predefined_mission(self):
        req = PreDefMissionRequest()
        if self.fw_square_mission_radio_button.isChecked():
            fw_client = navigator_client("wing")
            req.mission_type = "square_mission"
            print(
                f"{fw_client.name} Requesting for Upload Predefined Mission {req.mission_type} service!")
            response = fw_client.upload_predefined_mission_client(req)
            print("Request Result: %s" % response.accepted)
        if (self.tg_mission_radio_button.isEnabled()) and (self.tg_mission_radio_button.isChecked()):
            tg_client = navigator_client("target")
            req.mission_type = "target_mission"
            print(
                f"{tg_client.name} Requesting for Upload Predefined Mission {req.mission_type} service!")
            response = tg_client.upload_predefined_mission_client(req)
            print("Request Result: %s" % response.accepted)

    def update_custom_waypoint_table(self):
        row_count = self.fw_wp_table.rowCount()
        for row, wp in enumerate(self.custom_mission_list):
            if row < row_count:
                self.fw_wp_table.setItem(
                    row, 0, QtWidgets.QTableWidgetItem(str(wp.command)))
                self.fw_wp_table.setItem(
                    row, 1, QtWidgets.QTableWidgetItem(str(wp.x)))
                self.fw_wp_table.setItem(
                    row, 2, QtWidgets.QTableWidgetItem(str(wp.y)))
                self.fw_wp_table.setItem(
                    row, 3, QtWidgets.QTableWidgetItem(str(wp.z)))
            else:
                self.fw_wp_table.insertRow(row)
                self.fw_wp_table.setItem(
                    row, 0, QtWidgets.QTableWidgetItem(str(wp.command)))
                self.fw_wp_table.setItem(
                    row, 1, QtWidgets.QTableWidgetItem(str(wp.x)))
                self.fw_wp_table.setItem(
                    row, 2, QtWidgets.QTableWidgetItem(str(wp.y)))
                self.fw_wp_table.setItem(
                    row, 3, QtWidgets.QTableWidgetItem(str(wp.y)))

    def fw_clear_wp(self):
        self.custom_mission_list = []
        self.fw_wp_table.clearContents()

    def fw_add_wp(self):
        loc = LocationGlobal(float(self.fw_wp_lat.text()), float(
            self.fw_wp_lon.text()), float(self.fw_wp_alt.text()))
        wp = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, loc.lat, loc.lon, loc.alt)
        self.custom_mission_list.append(wp)
        self.update_custom_waypoint_table()

    def fw_upload_custom_mission(self):
        mission_list = self.custom_mission_list
        req = WP_list_uploadRequest()
        req.waypoints = []
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
            req.waypoints.append(wp)

        fw_client = navigator_client("wing")
        print(
            f"{fw_client.name} Requesting for over ros network upload mission file service!")
        print("Request Result: %s" % fw_client.upload_mission_ros_client(req))

    # TODO
    def fw_upload_mission_file_pickle(self):
        return


if __name__ == "__main__":
    rospy.init_node("clien_app")
    app = QtWidgets.QApplication(sys.argv)
    global last_fw_global_pos
    last_fw_global_pos = [35.41323864, 51.15932969, 1007]
    window = Ui()
    sys.exit(app.exec_())
