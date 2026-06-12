#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/kkzz730/roban-upper-body-control.git}"
REPO_DIR="${REPO_DIR:-$HOME/roban-upper-body-control}"
ROS_ACTIONS_DIR="${ROS_ACTIONS_DIR:-$HOME/robot_ros_application/catkin_ws/src/ros_actions_node}"

if [ -d "$REPO_DIR/.git" ]; then
    cd "$REPO_DIR"
    git pull --ff-only
else
    git clone "$REPO_URL" "$REPO_DIR"
    cd "$REPO_DIR"
fi

chmod +x scripts/yang/*.py scripts/yang/*.sh

if [ -d "$ROS_ACTIONS_DIR/scripts" ]; then
    mkdir -p "$ROS_ACTIONS_DIR/scripts/yang"
    cp scripts/yang/probe_bodyhub_interfaces.py "$ROS_ACTIONS_DIR/scripts/yang/"
    cp scripts/yang/monitor_servo_positions.py "$ROS_ACTIONS_DIR/scripts/yang/"
    cp scripts/yang/test_joint_controlpoint_publish.py "$ROS_ACTIONS_DIR/scripts/yang/"
    cp scripts/yang/week4_servo_upper_body_controller.py "$ROS_ACTIONS_DIR/scripts/yang/"
    cp scripts/yang/fake_pose_high_rate_publisher.py "$ROS_ACTIONS_DIR/scripts/yang/"
    cp scripts/yang/start_week4_servo_debug_tmux.sh "$ROS_ACTIONS_DIR/scripts/yang/"
    chmod +x "$ROS_ACTIONS_DIR/scripts/yang/"*.py "$ROS_ACTIONS_DIR/scripts/yang/"*.sh
fi

echo "Repo is ready at: $REPO_DIR"
echo "Start fake-pose hardware debug with:"
echo "  cd $REPO_DIR && scripts/yang/start_week4_servo_debug_tmux.sh"
