#!/usr/bin/env bash

sitl_command = "sim_vehicle.py -v ArduPlane -f gazebo-zephyr -m --mav10 -I0"
multi_vehicle_sim = 0

while getopts ":mcos" opt; do
  case "$opt" in
    m ) sitl_command="${sitl_command} --map"
    ;;
    c ) sitl_command="${sitl_command} --console"
    ;;
    o ) sitl_command="${sitl_command} --osd"
    ;;
    s ) multi_vehicle_sim = 1
    ;;
  esac
done
shift $((OPTIND -1))

xterm -hold -e ${sitl_command} &
# xterm -hold -e "gazebo --verbose /home/areza/Downloads/ardupilot_gazebo/worlds/wingpi.world"
if [[ $multi_vehicle_sim -eq 1 ]]; then
  xterm -hold -e "sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 --console -I1" &
fi


xterm -hold -e "gazebo --verbose $1"
