#!/bin/bash
# Launch the full autonomous stack on the Pi.  Ctrl-C stops everything.
#
#   ./launch_all.sh [n_balls] [cfg_side] [cfg_goal_sides]
#
# Prerequisites on the Pi: ROS 2 sourced, picamera2, gpiozero, vision_msgs,
# and the IMX500 detector node publishing /dummy_detections (see repo docs).
set -u
N_BALLS="${1:-2}"
CFG_SIDE="${2:-blue}"
CFG_GOALS="${3:-HR}"
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")"

pids=()
run() { python3 "$@" & pids+=($!); }
trap 'kill "${pids[@]}" 2>/dev/null' INT TERM EXIT

run "$REPO/ball_color/lane_node.py"
run "$REPO/ball_color/ball_color_node.py"
run "$REPO/ball_color/team_side.py"
run "$REPO/ball_color/arrival_node.py"
run "$HERE/goal_sides_node.py"
run "$HERE/drive_node.py"
run "$HERE/servo_node.py"
run "$HERE/mission_node.py" --ros-args \
    -p "n_balls:=$N_BALLS" -p "cfg_side:=$CFG_SIDE" \
    -p "cfg_goal_sides:=$CFG_GOALS"

wait
