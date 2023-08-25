#!/bin/bash
#xdotool key ctrl+alt+minus
#cd ~/airsim_ws/PX4-Autopilot
#make px4_sitl_default none_iris
terminator --new-tab -e "cd ${1}/PX4-Autopilot && make px4_sitl_default gazebo"
#terminator_pid=$!
#trap "kill -9 $terminator_pid" EXIT
#trap "kill -9 $terminator_pid" SIGINT