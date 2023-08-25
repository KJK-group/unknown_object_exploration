#!/usr/bin/env bash

catkin_ws_dir=${1:-"~/catkin_ws"}
px4_dir=${2:-"${catkin_ws_dir}/PX4-Autopilot"}
# TMUX setup
SESSION=${3:-drone}

# shellcheck source=../scripts/
source ./scripts/tmux-helpers.sh # load library functions

config=$(mktemp)
cat > "$config" <<EOF
set -g mouse on

setw -g mode-keys vi
set-option -s set-clipboard off
bind P paste-buffer
bind-key -T copy-mode-vi v send-keys -X begin-selection
bind-key -T copy-mode-vi y send-keys -X rectangle-toggle
unbind -T copy-mode-vi Enter
bind-key -T copy-mode-vi Enter send-keys -X copy-pipe-and-cancel 'xclip -se c -i'
bind-key -T copy-mode-vi MouseDragEnd1Pane send-keys -X copy-pipe-and-cancel 'xclip -se c -i'

set -g mouse-select-pane on

bind -n S-Up select-pane -U
bind -n S-Down select-pane -D
bind -n  S-Left select-pane -L
bind -n  S-Right select-pane -R


bind -n C-k select-pane -U
bind -n C-j select-pane -D
bind -n  C-h select-pane -L
bind -n  C-l select-pane -R
EOF

echo "tmux config is:"
cat "${config}"

echo "killing previous tmux session ..."

tmux kill-session -t "${SESSION}" #2>&1 /dev/null

tmux new -s "${SESSION}" -d

tmux source-file "${config}"


on-pane-create "bass source ${catkin_ws_dir}/devel/setup.bash"

# Pane commands
layout-2-2
echo "creating layouts and sending commands please wait :-)"

pane-cmd 0 "bass source ${catkin_ws_dir}/devel/setup.bash"

pane-cmd 1 "cd ${px4_dir} && make px4_sitl_default none_iris"

# argument handling
# if [ $# -eq 0 ];
# then
#     echo "Defaulting to workspace ${catkin_ws_dir}"
# else
#     if [ $# -eq 1 ];
#     then
#         echo "Defaulting to PX4 dir at ${px4_dir}"
#     else
#         echo "Using provided PX4 dir at ${px4_dir}"
#         px4_dir="${2}"
#     fi
#     echo "Using provided workspace ${1}"
#     catkin_ws_dir="${1}"
# fi

pane-cmd 2 "roslaunch uoe airsim.launch catkin_ws_path:=${catkin_ws_dir} px4_path:=${px4_dir}"

blocks=$(mktemp)
# ~/Documents/Packaged\ Unreal\ Projects/Blocks/LinuxNoEditor/Blocks/Binaries/Linux/Blocks-Linux-Debug &
# until pgrep -i "Blocks"; do sleep 1; done; sleep 5
cat > "${blocks}" <<EOF
sleep 3
echo "Launching airsim_ros_pkgs & rviz"
roslaunch uoe airsim_vis.launch
EOF

pane-cmd 3 "eval $(cat "${blocks}")"

# kill_airsim() {
#     if fuser -k 4560/tcp; then echo "killed"
#     fi
#     if fuser -k 14540/udp; then echo "killed"
#     fi
#     if killall Blocks-Linux-Debug;
#     then echo "killed simulation succesfully"
#     else echo "failed killing simulation"
#     fi
#     exit
# }
# trap kill_airsim SIGINT
# trap kill_airsim EXIT

# End in user pane 1
goto-pane-with-id 0
tmux attach -t "${SESSION}"
tmux kill-session -t "${SESSION}"