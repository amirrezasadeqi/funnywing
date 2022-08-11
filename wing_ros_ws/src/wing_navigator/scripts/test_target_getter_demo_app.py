#!/usr/bin/env python

import rospy
from PyQt5 import QtWidgets, uic
from wing_navigator.srv import *
import sys
import subprocess as sp

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
        uic.loadUi("/home/areza/.ros/demo_app/demo_app.ui", self)

        self.button = self.findChild(QtWidgets.QPushButton, 'pushButton')
        self.button.clicked.connect(self.pushButtonPressed)
        self.gzworld_button = self.findChild(QtWidgets.QPushButton, 'pushButton_3')
        self.gzworld_button.clicked.connect(self.get_gazebo_world_file)
        self.sim_button = self.findChild(QtWidgets.QPushButton, 'pushButton_2')
        self.sim_button.clicked.connect(self.run_simulation)
        
        
        self.lat = self.findChild(QtWidgets.QLineEdit, 'lineEdit')
        self.lon = self.findChild(QtWidgets.QLineEdit, 'lineEdit_2')
        self.alt = self.findChild(QtWidgets.QLineEdit, 'lineEdit_3')
        self.gz_world_address = self.findChild(QtWidgets.QLineEdit, 'lineEdit_4')
        self.sim_map_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox')
        self.sim_console_chbox = self.findChild(QtWidgets.QCheckBox, 'checkBox_2')

        self.show()

    def get_gazebo_world_file(self):
        worldfile, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Single File', "/home/areza/", '*.world')
        if worldfile:
            self.gz_world_address.setText(worldfile)

    def run_simulation(self):
        args = []
        args.append("/home/areza/Documents/funnywing/wing_ros_ws/src/wing_navigator/scripts/run_simulation.bash")
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



