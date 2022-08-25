#!/usr/bin/env python

import rospy
from PyQt5 import QtWidgets, uic
from wing_navigator.srv import *
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

def simple_goto_client(req):
  rospy.wait_for_service("/simple_goto")
  try:
    simple_goto_service = rospy.ServiceProxy("/simple_goto", SimpleGoto)
    response = simple_goto_service(req)
    return response.accepted
  except rospy.ServiceException as e:
    print("Service Call Failed: %s"%e)

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uilink_path = uilink_if_needed()
        uic.loadUi(uilink_path, self)

        self.button = self.findChild(QtWidgets.QPushButton, 'pushButton')
        self.button.clicked.connect(self.pushButtonPressed)
        
        
        self.lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit')
        self.lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_2')
        self.alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_3')

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

    def pushButtonPressed(self):
        # sp.check_call(["./demo_app.bash", self.lat.text(), self.lon.text(), self.alt.text()])
        req = SimpleGotoRequest()
        req.lat = float(self.lat.text())
        req.lon = float(self.lon.text())
        req.alt = float(self.alt.text())
        print("Requesting to for Simple goto service to point (lat:%s, lon:%s, alt:%s)"%(req.lat, req.lon, req.alt))
        print("Request Result: %s"%simple_goto_client(req))


if __name__ == "__main__":
    rospy.init_node("clien_app")
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()



