#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-roban_week4_servo}"
REPO_DIR="${REPO_DIR:-$HOME/roban-upper-body-control}"
CATKIN_WS="${CATKIN_WS:-$HOME/robot_ros_application/catkin_ws}"
SETUP_BASH="${SETUP_BASH:-$CATKIN_WS/devel/setup.bash}"
ACTION_SCRIPTS_DIR="${ACTION_SCRIPTS_DIR:-$CATKIN_WS/src/ros_actions_node/scripts}"
PYTHON_BIN="${PYTHON_BIN:-python}"

MODE="${MODE:-fake}"
INPUT_TOPIC="${INPUT_TOPIC:-/upper_body_pose_angles}"
JOINT_TOPIC="${JOINT_TOPIC:-/MediumSize/BodyHub/MotoPosition}"
CONTROLLER_MODE="${CONTROLLER_MODE:-legacy}"
JOINT_IDS="${JOINT_IDS:-14,15,17,18}"
ENABLED_ARMS="${ENABLED_ARMS:-both}"
POSE_MODE="${POSE_MODE:-both}"
CONTROL_ID="${CONTROL_ID:-2}"
REMOTE_JSON_PATH="${REMOTE_JSON_PATH:-/home/lemon/roban_motion_control/week4/logs/upper_body_pose_angles.json}"
REMOTE_JSON_DIR="$(dirname "$REMOTE_JSON_PATH")"

POSE_HZ="${POSE_HZ:-30}"
SERVO_HZ="${SERVO_HZ:-100}"
CONFIDENCE_THRESHOLD="${CONFIDENCE_THRESHOLD:-0.85}"
STALE_TIMEOUT="${STALE_TIMEOUT:-0.6}"
SOURCE_STALE_TIMEOUT="${SOURCE_STALE_TIMEOUT:-2.0}"
PRE_ACTION_HOME_DURATION="${PRE_ACTION_HOME_DURATION:-1.0}"
SHUTDOWN_HOME_DURATION="${SHUTDOWN_HOME_DURATION:-1.5}"
ALPHA="${ALPHA:-0.25}"
MAX_STEP_DEG="${MAX_STEP_DEG:-0.6}"

quote() {
    printf "%q" "$1"
}

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is required. Install it on the robot first, or run the three printed commands manually."
    exit 1
fi

if [ ! -d "$REPO_DIR" ]; then
    echo "Repo directory not found: $REPO_DIR"
    echo "Set REPO_DIR=/path/to/roban-upper-body-control and run again."
    exit 1
fi

if [ ! -f "$SETUP_BASH" ]; then
    echo "ROS setup file not found: $SETUP_BASH"
    echo "Set SETUP_BASH=/path/to/devel/setup.bash and run again."
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Attaching existing tmux session: $SESSION"
    if [ -t 0 ] && [ -t 1 ]; then
        exec tmux attach-session -t "$SESSION"
    fi
    exit 0
fi

COMMON_CMD="cd $(quote "$REPO_DIR") && source $(quote "$SETUP_BASH")"

if [ "$MODE" = "fake" ]; then
    POSE_CMD="$COMMON_CMD && $PYTHON_BIN scripts/yang/fake_pose_high_rate_publisher.py --topic $(quote "$INPUT_TOPIC") --hz $(quote "$POSE_HZ") --mode $(quote "$POSE_MODE")"
elif [ "$MODE" = "real" ]; then
    WAITING_JSON='{"visible": false, "confidence": 0.0, "left_arm_raise_angle": null, "left_elbow_angle": null, "right_arm_raise_angle": null, "right_elbow_angle": null, "reason": "waiting for 2hz mediapipe input"}'
    POSE_CMD="$COMMON_CMD && mkdir -p $(quote "$REMOTE_JSON_DIR") && printf '%s\n' $(quote "$WAITING_JSON") > $(quote "$REMOTE_JSON_PATH") && $PYTHON_BIN scripts/zhang/json_pose_publisher.py"
else
    echo "Unknown MODE=$MODE. Use MODE=fake or MODE=real."
    exit 1
fi

if [ "$CONTROLLER_MODE" = "legacy" ]; then
    CONTROLLER_CMD="$COMMON_CMD && export PYTHONPATH=$(quote "$ACTION_SCRIPTS_DIR"):\$PYTHONPATH && $PYTHON_BIN scripts/yang/week4_upper_body_controller.py"
    RATE_CMD="$COMMON_CMD && echo 'Measuring pose input rate on $INPUT_TOPIC' && rostopic hz $(quote "$INPUT_TOPIC")"
elif [ "$CONTROLLER_MODE" = "servo" ]; then
    CONTROLLER_CMD="$COMMON_CMD && $PYTHON_BIN scripts/yang/week4_servo_upper_body_controller.py --input-topic $(quote "$INPUT_TOPIC") --joint-topic $(quote "$JOINT_TOPIC") --joint-ids $(quote "$JOINT_IDS") --enabled-arms $(quote "$ENABLED_ARMS") --control-id $(quote "$CONTROL_ID") --prepare-bodyhub --hz $(quote "$SERVO_HZ") --confidence-threshold $(quote "$CONFIDENCE_THRESHOLD") --stale-timeout $(quote "$STALE_TIMEOUT") --source-stale-timeout $(quote "$SOURCE_STALE_TIMEOUT") --pre-action-home-duration $(quote "$PRE_ACTION_HOME_DURATION") --shutdown-home-duration $(quote "$SHUTDOWN_HOME_DURATION") --alpha $(quote "$ALPHA") --max-step-deg $(quote "$MAX_STEP_DEG")"
    RATE_CMD="$COMMON_CMD && echo 'Measuring JointControlPoint publish rate on $JOINT_TOPIC' && rostopic hz $(quote "$JOINT_TOPIC")"
else
    echo "Unknown CONTROLLER_MODE=$CONTROLLER_MODE. Use CONTROLLER_MODE=legacy or CONTROLLER_MODE=servo."
    exit 1
fi

tmux new-session -d -s "$SESSION" -n week4 -c "$REPO_DIR"
tmux send-keys -t "$SESSION:0.0" "$POSE_CMD" C-m

tmux split-window -h -t "$SESSION:0.0" -c "$REPO_DIR"
tmux send-keys -t "$SESSION:0.1" "$CONTROLLER_CMD" C-m

tmux split-window -v -t "$SESSION:0.1" -c "$REPO_DIR"
tmux send-keys -t "$SESSION:0.2" "$RATE_CMD" C-m

tmux select-pane -t "$SESSION:0.1"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

echo "Started tmux session: $SESSION"
echo "Pane 1: pose input, MODE=$MODE POSE_MODE=$POSE_MODE"
echo "Pane 2: controller, CONTROLLER_MODE=$CONTROLLER_MODE"
echo "Pane 3: rostopic hz monitor"
if [ -t 0 ] && [ -t 1 ]; then
    exec tmux attach-session -t "$SESSION"
fi
