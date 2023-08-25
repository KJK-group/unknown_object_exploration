select-pane() { tmux select-pane -t ${SESSION} -${1} ; }

pane-down() { select-pane D ; }
pane-up() { select-pane U ; }
pane-left() { select-pane L ; }
pane-right() { select-pane R ; }

send-tmux-cmd() { tmux send-keys -t ${1} "${2}" C-m ; }

TMUX_HELPERS_ON_PANE_CREATE=""

on-pane-create () {
	if [ -z "$TMUX_HELPERS_ON_PANE_CREATE"  ]; then
		TMUX_HELPERS_ON_PANE_CREATE="${1}"
	else
		TMUX_HELPERS_ON_PANE_CREATE="${TMUX_HELPERS_ON_PANE_CREATE} ; ${1}"
	fi
}

split-pane-along-axis() {
	axis=${1:-err}

	if [[ $err == "err" ]]; then
		return 1
	fi

	flag=""
	case ${axis} in
		h | hoz | horizontal)
			flag="v"
			;;
		v | vert | vertical)
			flag="h"
			;;
		*)
			return 1
			;;
	esac

	tmux split-pane -${flag} -t ${SESSION} -c ${1:-$HOME}
	send-tmux-cmd "${SESSION}" "eval ${TMUX_HELPERS_ON_PANE_CREATE}"
}


hsplit-pane() { split-pane-along-axis h ; }
vsplit-pane() { split-pane-along-axis v ; }

focus-pane-with-id() { tmux select-pane -t ${1:-0} ; }

pane-cmd() {
	pane=${1:-0}
	cmd=${2}
	delay=${3:-0}
	window=${4:-0}
	session=${5:-$SESSION}

	send-tmux-cmd "${session}:${window}.${pane}" "eval sleep ${delay}; ${cmd}"
}

layout-r-c () {
	r=${1:-1}
	c=${2:-1}

	for (( i = 1; i < r; i++ )); do
		hsplit-pane
	done

	for (( i = r - 1; i >= 0; i-- )); do
		focus-pane-with-id ${i}
		for (( j = 1; j < c; j++ )); do
			focus-pane-with-id ${i}
			vsplit-pane
		done
	done
	# tmux select-layout tiled
}


# for (( i = 2; i <= 4; i++ )); do
#     for (( j = 1; i <= 4; i++ )); do
#         eval "function layout-${i}-${j}() { layout-r-c ${i} ${j}; }"
#     done
# done

layout-1-2 () { layout-r-c 1 2; }
layout-1-3 () { layout-r-c 1 3; }
layout-1-4 () { layout-r-c 1 4; }
layout-2-1 () { layout-r-c 2 1; }
layout-2-2 () { layout-r-c 2 2; }
layout-2-3 () { layout-r-c 2 3; }
layout-2-4 () { layout-r-c 2 4; }
layout-3-1 () { layout-r-c 3 1; }
layout-3-2 () { layout-r-c 3 2; }
layout-3-3 () { layout-r-c 3 3; }
layout-3-4 () { layout-r-c 3 4; }
layout-4-1 () { layout-r-c 4 1; }
layout-4-2 () { layout-r-c 4 2; }
layout-4-3 () { layout-r-c 4 3; }
layout-4-4 () { layout-r-c 4 4; }

