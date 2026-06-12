# Week 4 Servo Upper-Body Controller Design

## Why replace action segment playback

The previous `week4_upper_body_controller.py` flow is useful as a safe fallback, but it is not suitable for high-frame-rate imitation. Each incoming pose JSON is converted into a full action segment and sent through `client_action.action(frames)`. While that segment is playing, the controller is effectively busy, new poses are skipped or delayed, and every segment returns to `BASE_FRAME`. At a 5-10Hz pose input rate this produces repeated start-stop motion instead of continuous tracking.

The new controller keeps the existing pose detection and JSON publishing pipeline unchanged. Only the robot-side controller changes.

## Control model

The real-time controller follows the same idea as a face-tracking servo loop:

```text
Mac pose detection / JSON output     5-10Hz real camera, 30Hz fake stress input
/upper_body_pose_angles ROS topic    5-10Hz real camera, 30Hz fake stress input
pose callback                        update latest target only
servo control loop                   100Hz JointControlPoint publish by default
Roban upper body                     smooth continuous tracking
```

On the tested robot, `rosmsg show bodyhub/JointControlPoint` reports:

```text
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
uint16 mainControlID
```

That means the controller cannot rely on `jointIdList`. In full-position mode it first reads `/MediumSize/BodyHub/ServoPositions`, preserves the current 22-joint frame for non-upper-body joints, and only overwrites the configured upper-body IDs.

The callback is intentionally small. It parses `std_msgs/String`, checks `visible` and `confidence`, maps human arm angles to four Roban upper-body joint targets, stores those targets in a thread-safe `TargetState`, and returns immediately.

The control loop runs independently at `--hz` and always publishes the latest smoothed joint command. If the pose is stale or invalid, it smoothly returns to the safe base pose.

## Joint mapping

The initial four joint IDs are based on the old 22-dimensional action-frame indices:

```text
list index 13,14,16,17 -> one-based servo ID 14,15,17,18
```

Default order:

```text
left_shoulder,left_elbow,right_shoulder,right_elbow = 14,15,17,18
```

These IDs should be confirmed on the robot with `monitor_servo_positions.py`. They can be overridden:

```bash
python scripts/yang/week4_servo_upper_body_controller.py \
  --joint-ids 14,15,17,18
```

Base angles:

```text
left_shoulder  = -61
left_elbow     = -18
right_shoulder =  61
right_elbow    =  18
```

Human elbow angle is converted to bend angle with `180 - elbow_angle`. Shoulder raise and elbow bend are both mapped conservatively into a 0-35 degree robot delta.

## Safety strategy

The controller includes several safety layers:

- `visible=false` returns the target to base pose.
- `confidence < --confidence-threshold` returns the target to base pose.
- No valid pose for `--stale-timeout` seconds returns to base pose and prints `pose stale, returning to base pose`.
- Every target angle is clamped by per-joint hard limits.
- Every control cycle applies low-pass filtering with `--alpha`.
- Every control cycle applies slew limiting with `--max-step-deg`.
- The default servo rate is 100Hz, with `--max-step-deg 0.6`, so the nominal max joint speed stays around 60 degrees per second.

Default joint limits:

```text
left_shoulder  -100 to 20
left_elbow      -70 to 30
right_shoulder  -20 to 100
right_elbow     -30 to 70
```

Limits can be adjusted without editing the core logic:

```bash
python scripts/yang/week4_servo_upper_body_controller.py \
  --joint-limits left_shoulder:-95:10,left_elbow:-65:25
```

## Robot-side debug flow

Probe actual BodyHub interfaces:

```bash
python scripts/yang/probe_bodyhub_interfaces.py
```

The probe writes:

```text
logs/week4/bodyhub_interface_probe.txt
```

Calibrate moving servo IDs:

```bash
python scripts/yang/monitor_servo_positions.py \
  --topic /MediumSize/BodyHub/ServoPositions
```

In another terminal, run a known arm action and record which IDs change.

Test one joint through `JointControlPoint`:

```bash
python scripts/yang/test_joint_controlpoint_publish.py \
  --topic /MediumSize/BodyHub/MotoPosition \
  --joint-id 17 \
  --base-angle 61 \
  --amplitude 5 \
  --hz 20 \
  --duration 5
```

Run fake high-rate pose input:

```bash
python scripts/yang/fake_pose_high_rate_publisher.py --hz 30
```

Run the new servo controller:

```bash
python scripts/yang/week4_servo_upper_body_controller.py \
  --input-topic /upper_body_pose_angles \
  --joint-topic /MediumSize/BodyHub/MotoPosition \
  --joint-ids 14,15,17,18 \
  --hz 100 \
  --confidence-threshold 0.85 \
  --stale-timeout 0.6 \
  --alpha 0.25 \
  --max-step-deg 0.6
```

Or start the fake-pose hardware debug setup in one tmux session with three panes:

```bash
cd ~/roban-upper-body-control
scripts/yang/start_week4_servo_debug_tmux.sh
```

The panes are:

- fake pose publisher at 30Hz, or input-rate monitor when `MODE=real`
- servo controller at 100Hz
- `rostopic hz` monitor for the JointControlPoint topic

For the real Mac pose pipeline instead of fake poses:

```bash
MODE=real scripts/yang/start_week4_servo_debug_tmux.sh
```

The default debug script is set up for the first hardware check where the person raises the right arm and the left arm should stay still:

```text
POSE_MODE=right-only
ENABLED_ARMS=right
```

In this mode fake JSON only changes the right-arm angles, and the controller only overwrites the robot right shoulder and right elbow. The left arm keeps the startup servo angles read from `/MediumSize/BodyHub/ServoPositions`.

For both-arm control:

```bash
ENABLED_ARMS=both POSE_MODE=both scripts/yang/start_week4_servo_debug_tmux.sh
```

Useful overrides:

```bash
JOINT_TOPIC=/actual/JointControlPoint/topic \
JOINT_IDS=14,15,17,18 \
ENABLED_ARMS=right \
SERVO_HZ=100 \
POSE_HZ=30 \
scripts/yang/start_week4_servo_debug_tmux.sh
```

## Acceptance checks

- The new controller does not call action-segment playback.
- It publishes `bodyhub/JointControlPoint` messages at 100Hz by default, or the highest stable rate confirmed by `rostopic hz` on the robot.
- `/upper_body_pose_angles` can update at 5-10Hz without blocking the callback.
- The robot follows in small continuous steps instead of playing one complete action per pose.
- Invalid, low-confidence, or stale pose data returns the upper body to base pose.
- The terminal status prints every 0.5 seconds, not every frame.

Recommended evidence to capture during testing:

- `rostopic hz /upper_body_pose_angles`, expected around 5-10Hz with the real camera path or 30Hz with the fake publisher.
- `rostopic hz /MediumSize/BodyHub/MotoPosition` or the actual JointControlPoint topic, expected near 100Hz if BodyHub and ROS scheduling keep up.
- A short video or screenshot showing continuous arm tracking with fake pose input.
