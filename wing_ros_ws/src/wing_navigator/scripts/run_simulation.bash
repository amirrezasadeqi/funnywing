#!/usr/bin/env bash

sitl_command="sim_vehicle.py -v ArduPlane -f gazebo-zephyr -m --mav10 -I0"
multi_vehicle_sim=0

while getopts ":mcos" opt; do
  case "$opt" in
    m ) sitl_command="${sitl_command} --map"
    ;;
    c ) sitl_command="${sitl_command} --console"
    ;;
    o ) sitl_command="${sitl_command} --osd"
    ;;
    s ) multi_vehicle_sim=1
    ;;
  esac
done
shift $((OPTIND -1))
loc=$1
world_file=$2
sitl_command="${sitl_command} -L ${loc}"

xterm -hold -e ${sitl_command} &
if [[ $multi_vehicle_sim -eq 1 ]]; then
  xterm -hold -e "sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 --console --map -I1 -L ${loc}" &
fi


xterm -hold -e "gazebo --verbose ${world_file}" &

# This exit function and the previous & command suddenly found by GOD and I Solved the problems
# "is not responding errors". I think by not exiting this bash script the application python script
# locked(maybe a mutex was being locked) and the program stuck at that error. Thanks GOD!
exit
