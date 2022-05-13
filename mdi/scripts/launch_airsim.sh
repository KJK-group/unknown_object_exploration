#!/bin/bash
catkin_ws_dir="~/catkin_ws"
px4_dir="${catkin_ws_dir}/PX4-Autopilot"
if [ $# -eq 0 ];
then
    echo "Defaulting to workspace ${catkin_ws_dir}"
else
    if [ $# -eq 1 ];
    then
        echo "Defaulting to PX4 dir at ${px4_dir}"
    else
        echo "Using provided PX4 dir at ${px4_dir}"
        px4_dir="${2}"
    fi
    echo "Using provided workspace ${1}"
    catkin_ws_dir="${1}"
fi
roslaunch mdi airsim.launch catkin_ws_path:=${catkin_ws_dir} &
rl_pid=$!
trap "kill -9 $rl_pid" SIGINT
trap "kill -9 $rl_pid" EXIT

until pgrep -i "Blocks";
do
sleep 1
done
sleep 5
echo "Launching airsim_ros_pkgs & rviz"
roslaunch mdi airsim_vis.launch

kill_airsim() {
    if fuser -k 4560/tcp; then echo "killed"
    fi
    if fuser -k 14540/udp; then echo "killed"
    fi
    if killall Blocks-Linux-Debug;
    then echo "killed simulation succesfully"
    else echo "failed killing simulation"
    fi
}
trap kill_airsim SIGINT
trap kill_airsim EXIT