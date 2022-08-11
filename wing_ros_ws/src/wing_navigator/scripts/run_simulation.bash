#!/usr/bin/env bash

xterm -hold -e "sim_vehicle.py -v ArduPlane -f gazebo-zephyr -m --mav10 --map --console -I0" &
# xterm -hold -e "gazebo --verbose /home/areza/Downloads/ardupilot_gazebo/worlds/wingpi.world"
xterm -hold -e "gazebo --verbose $1"

