#!/usr/bin/env python

import rospy
from PyQt5 import QtWidgets, uic
from wing_navigator.srv import SimpleGoto, SimpleGotoRequest, ActiveMode, ActiveModeRequest, ArmTakeoff, ArmTakeoffRequest #, SimpleGotoResponse
import sys
import subprocess as sp
from os.path import expanduser, exists
from os import symlink, makedirs

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

def active_mode_client(req):
    rospy.wait_for_service("/active_mode")
    try:
        active_mode_service = rospy.ServiceProxy("/active_mode", ActiveMode)
        response = active_mode_service(req)
        return response.accepted
    except rospy.ServiceException as e:
        print("Service Call Failed: %s"%e)

def simple_goto_client(req):
  rospy.wait_for_service("/simple_goto")
  try:
    simple_goto_service = rospy.ServiceProxy("/simple_goto", SimpleGoto)
    response = simple_goto_service(req)
    return response.accepted
  except rospy.ServiceException as e:
    print("Service Call Failed: %s"%e)

def arm_takeoff_client(req):
  rospy.wait_for_service("/arm_takeoff")
  try:
    arm_takeoff_service = rospy.ServiceProxy("/arm_takeoff", ArmTakeoff)
    response = arm_takeoff_service(req)
    return response.accepted
  except rospy.ServiceException as e:
    print("Service Call Failed: %s"%e)

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uilink_path = uilink_if_needed()
        uic.loadUi(uilink_path, self)

        # Simulation Tab objects
        self.gz_world_address = self.findChild(QtWidgets.QLineEdit, 'lineEdit_4')
        self.sim_map_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox')
        self.sim_console_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox_2')
        self.sim_multi_vehicle_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox_3')
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
        self.fw_save_current_mission_button.clicked.connect(self.fw_save_current_mission)
        self.fw_upload_mission_file_button = self.findChild(QtWidgets.QPushButton, 'pushButton_8')
        self.fw_upload_mission_file_button.clicked.connect(self.fw_upload_mission_file)
        
        ### Pre-Defined Missions
        self.fw_square_mission_radio_button = self.findChild(QtWidgets.QRadioButton, 'radioButton')
        self.tg_mission_radio_button = self.findChild(QtWidgets.QRadioButton, 'radioButton_2')
        self.upload_predefined_mission_button = self.findChild(QtWidgets.QPushButton, 'pushButton_9')
        self.upload_predefined_mission_button.clicked.connect(self.upload_predefined_mission)

        ## Custom Missions
        self.fw_wp_lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit_5')
        self.fw_wp_lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_6')
        self.fw_wp_alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_7')
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
        print("Request Result: %s"%active_mode_client(req))

    def fw_arm_takeoff(self):
        req = ArmTakeoffRequest()
        print("Request for Arm and Takeoff vehicle")
        print("Request Result: %s"%arm_takeoff_client(req))

    def fw_return_home(self):
        req = ActiveModeRequest("RTL")
        # req.mode = bytes(self.fw_mode_name_combo_box.currentText().encode('utf-8'))
        print("Request for going back to home")
        print("Request Result: %s"%active_mode_client(req))

    def fw_gotoButtonPressed(self):
        # sp.check_call(["./demo_app.bash", self.lat.text(), self.lon.text(), self.alt.text()])
        req = SimpleGotoRequest()
        req.lat = float(self.fw_goto_lat.text())
        req.lon = float(self.fw_goto_lon.text())
        req.alt = float(self.fw_goto_alt.text())
        print("Requesting to for Simple goto service to point (lat:%s, lon:%s, alt:%s)"%(req.lat, req.lon, req.alt))
        print("Request Result: %s"%simple_goto_client(req))

    def tg_active_mode(self):
        return
    def tg_arm_takeoff(self):
        return
    def tg_return_home(self):
        return
    def tg_gotoButtonPressed(self):
        # sp.check_call(["./demo_app.bash", self.lat.text(), self.lon.text(), self.alt.text()])
        req = SimpleGotoRequest()
        req.lat = float(self.tg_goto_lat.text())
        req.lon = float(self.tg_goto_lon.text())
        req.alt = float(self.tg_goto_alt.text())
        print("Target Requesting to for Simple goto service to point (lat:%s, lon:%s, alt:%s)"%(req.lat, req.lon, req.alt))
        print("Target Request Result: %s"%simple_goto_client(req))

    def get_fw_mission_file(self):
        return
    def fw_save_current_mission(self):
        return
    def fw_upload_mission_file(self):
        return
    def upload_predefined_mission(self):
        return
    def fw_clear_wp(self):
        return
    def fw_add_wp(self):
        return
    def fw_upload_custom_mission(self):
        return

if __name__ == "__main__":
    rospy.init_node("clien_app")
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()



