#!/usr/bin/env python

### A script to setup system files to run simulation ###
import os
from shutil import copyfile
# import git


def uilink_if_needed():
    # Address of symlink to ui file
    uilink_path = os.path.expanduser("~/.ros/demo_app/demo_app.ui")
    # Address of the symlink directory
    uilink_dir = os.path.expanduser("~/.ros/demo_app")
    # Address of the original ui file. Considered to be located at the ~/Documents/funnywing/...
    uifile_path = os.path.expanduser(
        "~/Documents/funnywing/wing_ros_ws/src/wing_navigator/scripts/demo_app.ui")
    if not os.path.exists(uilink_path):
        if not os.path.exists(uilink_dir):
            os.makedirs(uilink_dir)
        os.symlink(uifile_path, uilink_path)

    return uilink_path


if __name__ == "__main__":

    # Check if the ardupilot_gazebo exists and if not clone it
    if os.path.exists(os.path.expanduser("~/Downloads/ardupilot_gazebo/")):
        print("!!!Note!!!: You may need to change the fdm ports in the models of this repository")
        if not os.path.exists(os.path.expanduser("~/Downloads/ardupilot_gazebo/worlds/wingpi.world")):
            copyfile(os.path.expanduser("~/Documents/funnywing/wing_ros_ws/src/wing_navigator/worlds/wingpi.world"),
                     os.path.expanduser("~/Downloads/ardupilot_gazebo/worlds/wingpi.world"))
        if not os.path.exists(os.path.expanduser("~/Downloads/ardupilot_gazebo/worlds/wingpi_multidrone.world")):
            copyfile(os.path.expanduser("~/Documents/funnywing/wing_ros_ws/src/wing_navigator/worlds/wingpi_multidrone.world"),
                     os.path.expanduser("~/Downloads/ardupilot_gazebo/worlds/wingpi_multidrone.world"))
    else:
        print("Cloning SwiftGust ardupilot_gazebo repository for simulation:")
        os.system(
            "git clone https://github.com/SwiftGust/ardupilot_gazebo.git ~/Downloads/")
        print("!!!Note!!!: You may need to change the fdm ports in the models of this repository")
        copyfile(os.path.expanduser("~/Documents/funnywing/wing_ros_ws/src/wing_navigator/worlds/wingpi.world"),
                 os.path.expanduser("~/Downloads/ardupilot_gazebo/worlds/wingpi.world"))
        copyfile(os.path.expanduser("~/Documents/funnywing/wing_ros_ws/src/wing_navigator/worlds/wingpi_multidrone.world"),
                 os.path.expanduser("~/Downloads/ardupilot_gazebo/worlds/wingpi_multidrone.world"))

    if not os.path.exists(os.path.expanduser("~/.config/ardupilot/locations.txt")):
        copyfile(os.path.expanduser("~/Documents/funnywing/locations.txt"),
                 os.path.expanduser("~/.config/ardupilot/locations.txt"))

    # ui file symbolic linking
    _ = uilink_if_needed()
