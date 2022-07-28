#!/usr/bin/env bash

xterm -hold -e "/home/areza/Downloads/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-zephyr -m --mav10 --map --console -I0" &
xterm -hold -e "gazebo --verbose /home/areza/Downloads/SwiftGust/ardupilot_gazebo/worlds/zephyr_ardupilot_demo.world"

