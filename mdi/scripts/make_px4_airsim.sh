#!/usr/bin/env bash

SESSION=${2:-drone}

source ./tmux-helpers.sh # load library functions

config=$(mktemp)
cat > $config <<EOF
set -g mouse on
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
cat $config

echo "killing previous tmux session ..."

tmux kill-session -t ${SESSION} #2>&1 /dev/null

tmux new -s ${SESSION} -d

tmux source-file ${config}

on-pane-create "bass source (catkin locate)"
# on-pane-create "ls"
# on-pane-create "ip"


# create desired layout
layout-1-2

echo "creating layouts and sending commands please wait :-)"

pane-cmd 2 "cd ${1} && make px4_sitl_default none_iris"

goto-pane-with-id 1

tmux attach -t ${SESSION}