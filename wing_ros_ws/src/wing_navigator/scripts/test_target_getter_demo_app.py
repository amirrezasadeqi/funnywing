#!/usr/bin/env python

import rospy
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from wing_navigator.srv import PreDefMissionRequest, SimpleGoto, SimpleGotoRequest, ActiveMode, ActiveModeRequest, ArmTakeoff, ArmTakeoffRequest, MissionInOut, MissionInOutRequest #, SimpleGotoResponse
from wing_modules.navigator_modules.navigator_client import navigator_client
import sys
import subprocess as sp
from os.path import expanduser, exists
from os import symlink, makedirs
from pymavlink import mavutil
#### Test for custom service for mission ####
from wing_navigator.srv import WP_list_save, WP_list_saveRequest, WP_list_upload, WP_list_uploadRequest
from dronekit import Command, LocationGlobal
from wing_navigator.msg import MissionCommand
#############################################
### TODO: Experimental Usage of pickle ############
import pickle
from wing_navigator.srv import TestPickleUpload, TestPickleUploadRequest
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
        print("Request Result: %s"%tg_client.arm_takeoff_client(req))
        self.finished.emit()

#############################################################################

def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

def uilink_if_needed():
    # Address of symlink to ui file
    uilink_path = expanduser("~/.ros/demo_app/demo_app.ui")
    # Address of the symlink directory
    uilink_dir = expanduser("~/.ros/demo_app")
    # Address of the original ui file. Considered to be located at the ~/Documents/funnywing/...
    uifile_path = expanduser("~/Documents/funnywing/wing_ros_ws/src/wing_navigator/scripts/demo_app.ui")
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

        # fixed wing client objects
        self.fw_client = navigator_client("wing")

        # Adding A mission list to save custom waypoints
        self.custom_mission_list = []

        # Simulation Tab objects
        self.gz_world_address = self.findChild(QtWidgets.QLineEdit, 'lineEdit_4')
        self.sim_map_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox')
        self.sim_console_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox_2')
        self.sim_multi_vehicle_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox_3')
        self.sim_multi_vehicle_chbox.stateChanged.connect(self.sim_multi_vehicle_chbox_toggle_handler)
        self.sim_osd_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox_4')
        self.sim_loc_combo = self.findChild(QtWidgets.QComboBox, 'comboBox')
        self.gzworld_button = self.findChild(QtWidgets.QPushButton, 'pushButton_3')
        self.gzworld_button.clicked.connect(self.get_gazebo_world_file)
        self.sim_button = self.findChild(QtWidgets.QPushButton, 'pushButton_2')
        self.sim_button.clicked.connect(self.run_simulation)

        # Navigation Center Tab objects
        ## Fixed-Wing Navigation Dashboard
        ### Mode and Arm/Takeoff ...
        # self.fw_mode_name_text_in = self.findChild(QtWidgets.QLineEdit, 'lineEdit_9')
        self.fw_mode_name_combo_box = self.findChild(QtWidgets.QComboBox, 'comboBox_2')
        self.fw_active_mode_button = self.findChild(QtWidgets.QPushButton, 'pushButton_11')
        self.fw_active_mode_button.clicked.connect(self.fw_active_mode)
        self.fw_arm_takeoff_button = self.findChild(QtWidgets.QPushButton, 'pushButton_12')
        self.fw_arm_takeoff_button.clicked.connect(self.fw_arm_takeoff)
        self.fw_return_home_button = self.findChild(QtWidgets.QPushButton, 'pushButton_13')
        self.fw_return_home_button.clicked.connect(self.fw_return_home)

        ### Goto Service
        self.fw_goto_button = self.findChild(QtWidgets.QPushButton, 'pushButton')
        self.fw_goto_button.clicked.connect(self.fw_gotoButtonPressed)
        
        self.fw_goto_lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit')
        self.fw_goto_lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_2')
        self.fw_goto_alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_3')

        ## Target Navigation Dashboard
        ### Mode and Arm/Takeoff ...
        self.tg_mode_name_text_in = self.findChild(QtWidgets.QLineEdit, 'lineEdit_17')
        self.tg_mode_name_combo_box = self.findChild(QtWidgets.QComboBox, 'comboBox_4')
        self.tg_active_mode_button = self.findChild(QtWidgets.QPushButton, 'pushButton_21')
        self.tg_active_mode_button.clicked.connect(self.tg_active_mode)
        self.tg_arm_takeoff_button = self.findChild(QtWidgets.QPushButton, 'pushButton_19')
        self.tg_arm_takeoff_button.clicked.connect(self.tg_arm_takeoff)
        self.tg_return_home_button = self.findChild(QtWidgets.QPushButton, 'pushButton_20')
        self.tg_return_home_button.clicked.connect(self.tg_return_home)

        ### Goto Service
        self.tg_goto_button = self.findChild(QtWidgets.QPushButton, 'pushButton_18')
        self.tg_goto_button.clicked.connect(self.tg_gotoButtonPressed)
        
        self.tg_goto_lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit_15')
        self.tg_goto_lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_14')
        self.tg_goto_alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_16')

        # Mission Management Tab objects
        ## Import/Export and Pre-Defined Missions
        ### Mission File
        self.fw_mission_file_address = self.findChild(QtWidgets.QLineEdit, 'lineEdit_8')
        self.fw_mission_file_browse_button = self.findChild(QtWidgets.QPushButton, 'pushButton_6')
        self.fw_mission_file_browse_button.clicked.connect(self.get_fw_mission_file)
        self.fw_save_current_mission_button = self.findChild(QtWidgets.QPushButton, 'pushButton_7')
        # self.fw_save_current_mission_button.clicked.connect(self.fw_save_current_mission)
        self.fw_save_current_mission_button.clicked.connect(self.fw_save_current_mission_ros)
        self.fw_upload_mission_file_button = self.findChild(QtWidgets.QPushButton, 'pushButton_8')
        # self.fw_upload_mission_file_button.clicked.connect(self.fw_upload_mission_file)
        self.fw_upload_mission_file_button.clicked.connect(self.fw_upload_mission_file_ros)
        ###### TODO: Experimental upload using pickle ########################
        # self.fw_upload_mission_file_button.clicked.connect(self.fw_upload_mission_file_pickle)
        ################################################################
        
        ### Pre-Defined Missions
        self.fw_square_mission_radio_button = self.findChild(QtWidgets.QRadioButton, 'radioButton')
        self.tg_mission_radio_button = self.findChild(QtWidgets.QRadioButton, 'radioButton_2')
        if not self.sim_multi_vehicle_chbox.isChecked():
            self.tg_mission_radio_button.setEnabled(False)
        self.upload_predefined_mission_button = self.findChild(QtWidgets.QPushButton, 'pushButton_9')
        self.upload_predefined_mission_button.clicked.connect(self.upload_predefined_mission)

        ## Custom Missions
        self.fw_wp_lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit_5')
        self.fw_wp_lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_6')
        self.fw_wp_alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_7')
        self.fw_wp_table = self.findChild(QtWidgets.QTableWidget, 'tableWidget')
        #Table will fit the screen horizontally
        self.fw_wp_table.horizontalHeader().setStretchLastSection(True)
        self.fw_wp_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self.fw_wp_table.verticalHeader().setStretchLastSection(True)
        self.fw_wp_table.verticalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)

        self.fw_clear_wp_button = self.findChild(QtWidgets.QPushButton, 'pushButton_4')
        self.fw_clear_wp_button.clicked.connect(self.fw_clear_wp)
        self.fw_add_wp_button = self.findChild(QtWidgets.QPushButton, 'pushButton_5')
        self.fw_add_wp_button.clicked.connect(self.fw_add_wp)
        self.fw_upload_custom_mission_button = self.findChild(QtWidgets.QPushButton, 'pushButton_10')
        self.fw_upload_custom_mission_button.clicked.connect(self.fw_upload_custom_mission)


        self.show()

    def get_gazebo_world_file(self):
        worldfile, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Single File', expanduser("~"), '*.world')
        if worldfile:
            self.gz_world_address.setText(worldfile)

    def run_simulation(self):
        args = []
        args.append(expanduser("~/Documents/funnywing/wing_ros_ws/src/wing_navigator/scripts/run_simulation.bash"))

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
        print(f"Requesting to for Mode Activation service for Activating the {req.mode} Flight mode.")
        # print("Request Result: %s"%active_mode_client(req))
        print("Request Result: %s"%self.fw_client.active_mode_client(req))

    def fw_arm_takeoff(self):
        req = ArmTakeoffRequest()
        print(f"{self.fw_client} Requests for Arm and Takeoff vehicle")
        # print("Request Result: %s"%arm_takeoff_client(req))
        print("Request Result: %s"%self.fw_client.arm_takeoff_client(req))

    def fw_return_home(self):
        req = ActiveModeRequest("RTL")
        # req.mode = bytes(self.fw_mode_name_combo_box.currentText().encode('utf-8'))
        print(f"{self.fw_client} Requests for going back to home")
        # print("Request Result: %s"%active_mode_client(req))
        print("Request Result: %s"%self.fw_client.active_mode_client(req))

    def fw_gotoButtonPressed(self):
        # sp.check_call(["./demo_app.bash", self.lat.text(), self.lon.text(), self.alt.text()])
        req = SimpleGotoRequest()
        req.lat = float(self.fw_goto_lat.text())
        req.lon = float(self.fw_goto_lon.text())
        req.alt = float(self.fw_goto_alt.text())
        print(f"{self.fw_client} is Requesting for Simple goto service to point (lat:%s, lon:%s, alt:%s)"%(req.lat, req.lon, req.alt))
        print("Request Result: %s"%self.fw_client.simple_goto_client(req))

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
        print(f"Requesting to for Mode Activation service for Activating the {req.mode} Flight mode.")
        # print("Request Result: %s"%active_mode_client(req))
        print("Request Result: %s"%tg_client.active_mode_client(req))
        
    # def tg_arm_takeoff(self):
    #     req = ArmTakeoffRequest()
    #     tg_client = navigator_client("target")
    #     print(f"{tg_client.name} Requests for Arm and Takeoff vehicle")
    #     # print("Request Result: %s"%arm_takeoff_client(req))
    #     print("Request Result: %s"%tg_client.arm_takeoff_client(req))

    # Using QThread to prevent GUI freezes
    def tg_arm_takeoff(self):
        self.thread = QThread()
        self.tg_client_worker = client_worker()
        self.tg_client_worker.moveToThread(self.thread)
        self.thread.started.connect(self.tg_client_worker.run)
        self.thread.finished.connect(self.thread.quit)
        self.tg_client_worker.finished.connect(self.tg_client_worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()

        # TODO: This reset does not work currently because dronekit takeoff does not compelete
        # and I think the thread does not finish. Solve it later! for now just reactivate the
        # button on other events.
        # disable the button arm and takeoff button to prevent wrong click events
        # self.tg_arm_takeoff_button.setEnabled(False)
        # self.thread.finished.connect(
        #         # activate the button after thread job is done.
        #         lambda : self.tg_arm_takeoff_button.setEnabled(True)
        #         )

    def tg_return_home(self):
        req = ActiveModeRequest("RTL")
        tg_client = navigator_client("target")
        # req.mode = bytes(self.fw_mode_name_combo_box.currentText().encode('utf-8'))
        print(f"{tg_client.name} Requests for going back to home")
        # print("Request Result: %s"%active_mode_client(req))
        print("Request Result: %s"%tg_client.active_mode_client(req))

    def tg_gotoButtonPressed(self):
        # sp.check_call(["./demo_app.bash", self.lat.text(), self.lon.text(), self.alt.text()])
        req = SimpleGotoRequest()
        tg_client = navigator_client("target")
        req.lat = float(self.tg_goto_lat.text())
        req.lon = float(self.tg_goto_lon.text())
        req.alt = float(self.tg_goto_alt.text())
        print("Target Requesting to for Simple goto service to point (lat:%s, lon:%s, alt:%s)"%(req.lat, req.lon, req.alt))
        print("Target Request Result: %s"%tg_client.simple_goto_client(req))

    def get_fw_mission_file(self):
        fw_mission_file, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Single File', expanduser("~"), '*.txt')
        if fw_mission_file:
            self.fw_mission_file_address.setText(fw_mission_file)

    def fw_save_current_mission(self):
        req = MissionInOutRequest()
        fw_client = navigator_client("wing")
        req.filename = self.fw_mission_file_address.text()
        print("Requesting to for save current mission service")
        print("Request Result: %s"%fw_client.save_mission_client(req))

    def fw_save_current_mission_ros(self):
        req = WP_list_saveRequest()
        fw_client = navigator_client("wing")
        req.req_message = f"{fw_client.name} requests to save the current vehicle mission to local file: {self.fw_mission_file_address.text()}"
        #Fetching mission from vehicle with custom service format
        response = fw_client.save_mission_ros_client(req)
        mission_list_resp = response.waypoints
        # print("Request Result: %s"%tg_client.save_mission_client(req))
        ####################################################################
        #Add file-format information
        output='QGC WPL 110\n'
        #Add home location as 0th waypoint
        home_lat = response.home_lat
        home_lon = response.home_lon
        home_alt = response.home_alt
        output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home_lat,home_lon,home_alt,1)
        #Add commands
        for cmd in mission_list_resp:
            # commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
            commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.ln_2, cmd.ln_currentwp, cmd.ln_frame, cmd.ln_command, cmd.ln_param1, cmd.ln_param2, cmd.ln_param3, cmd.ln_param4, cmd.ln_param5, cmd.ln_param6, cmd.ln_param7, cmd.ln_autocontinue)
            output+=commandline

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
        print("Request Result: %s"%tg_client.upload_mission_client(req))

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
        print(f"{fw_client.name} Requesting for over ros network upload mission file service!")
        print("Request Result: %s"%fw_client.upload_mission_ros_client(req))

    def upload_predefined_mission(self):
        req = PreDefMissionRequest()
        if self.fw_square_mission_radio_button.isChecked():
            fw_client = navigator_client("wing")
            req.mission_type = "square_mission"
            print(f"{fw_client.name} Requesting for Upload Predefined Mission {req.mission_type} service!")
            response = fw_client.upload_predefined_mission_client(req)
            print("Request Result: %s"%response.accepted)
        if (self.tg_mission_radio_button.isEnabled()) and (self.tg_mission_radio_button.isChecked()):
            tg_client = navigator_client("target")
            req.mission_type = "target_mission"
            print(f"{tg_client.name} Requesting for Upload Predefined Mission {req.mission_type} service!")
            response = tg_client.upload_predefined_mission_client(req)
            print("Request Result: %s"%response.accepted)

    def update_custom_waypoint_table(self):
        row_count = self.fw_wp_table.rowCount()
        for row, wp in enumerate(self.custom_mission_list):
            if row < row_count:
                self.fw_wp_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(wp.command)))
                self.fw_wp_table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(wp.x)))
                self.fw_wp_table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(wp.y)))
                self.fw_wp_table.setItem(row, 3, QtWidgets.QTableWidgetItem(str(wp.z)))
            else:
                self.fw_wp_table.insertRow(row)
                self.fw_wp_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(wp.command)))
                self.fw_wp_table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(wp.x)))
                self.fw_wp_table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(wp.y)))
                self.fw_wp_table.setItem(row, 3, QtWidgets.QTableWidgetItem(str(wp.y)))


    def fw_clear_wp(self):
        self.custom_mission_list = []
        self.fw_wp_table.clearContents()

    def fw_add_wp(self):
        loc = LocationGlobal(float(self.fw_wp_lat.text()), float(self.fw_wp_lon.text()), float(self.fw_wp_alt.text()))
        wp = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, loc.lat, loc.lon, loc.alt)
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
        print(f"{fw_client.name} Requesting for over ros network upload mission file service!")
        print("Request Result: %s"%fw_client.upload_mission_ros_client(req))

    # TODO
    def fw_upload_mission_file_pickle(self):
        return

if __name__ == "__main__":
    rospy.init_node("clien_app")
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    sys.exit(app.exec_())



