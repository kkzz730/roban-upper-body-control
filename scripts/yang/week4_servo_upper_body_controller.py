#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import json
import threading
import time

import rospy
from bodyhub.msg import JointControlPoint
from bodyhub.msg import ServoPositionAngle
from std_msgs.msg import String


JOINT_NAMES = ["left_shoulder", "left_elbow", "right_shoulder", "right_elbow"]

BASE_FRAME = [
    0.0, -1.81067, 19.2848, -34.5006, -15.2158, -1.81067,
    0.0, 1.81067, -19.2848, 34.5006, 15.2158, 1.81067,
    0.0, -70.0, -15.0, 0.0, 70.0, 15.0, 0.0, 0.0, 0.0, 0.0,
]

BASE_ANGLES = {
    "left_shoulder": -70.0,
    "left_elbow": -15.0,
    "right_shoulder": 70.0,
    "right_elbow": 15.0,
}

DEFAULT_JOINT_IDS = {
    "left_shoulder": 14,
    "left_elbow": 15,
    "right_shoulder": 17,
    "right_elbow": 18,
}

JOINT_LIMITS = {
    "left_shoulder": (-100.0, 20.0),
    "left_elbow": (-70.0, 30.0),
    "right_shoulder": (-20.0, 100.0),
    "right_elbow": (-30.0, 70.0),
}

DEFAULT_INPUT_TOPIC = "/upper_body_pose_angles"
DEFAULT_JOINT_TOPIC = "/MediumSize/BodyHub/MotoPosition"
DEFAULT_SERVO_POSITION_TOPIC = "/MediumSize/BodyHub/ServoPositions"
DEFAULT_SERVICE = "/MediumSize/BodyHub/GetMasterID"
DEFAULT_CONTROL_HZ = 100.0
DEFAULT_MAX_STEP_DEG = 1.0
DEFAULT_CONTROL_ID = 2

ARM_JOINTS = {
    "left": ["left_shoulder", "left_elbow"],
    "right": ["right_shoulder", "right_elbow"],
    "both": JOINT_NAMES[:],
}


class TargetState(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.visible = False
        self.confidence = 0.0
        self.timestamp = 0.0
        self.pose = {}
        self.target_angles = BASE_ANGLES.copy()

    def update(self, visible, confidence, pose, target_angles):
        with self.lock:
            self.visible = visible
            self.confidence = confidence
            self.timestamp = time.time()
            self.pose = pose.copy()
            self.target_angles = target_angles.copy()

    def snapshot(self):
        with self.lock:
            return {
                "visible": self.visible,
                "confidence": self.confidence,
                "timestamp": self.timestamp,
                "pose": self.pose.copy(),
                "target_angles": self.target_angles.copy(),
            }


def clamp(value, low, high):
    return max(low, min(high, value))


def map_range(x, in_min, in_max, out_min, out_max):
    x = clamp(x, in_min, in_max)
    return out_min + (x - in_min) * (out_max - out_min) / float(in_max - in_min)


def low_pass(current, target, alpha):
    return current + alpha * (target - current)


def slew_limit(current, target, max_step):
    diff = target - current
    if diff > max_step:
        return current + max_step
    if diff < -max_step:
        return current - max_step
    return target


def clamp_angles(angles, limits):
    clamped = {}
    for name in JOINT_NAMES:
        low, high = limits[name]
        clamped[name] = clamp(angles[name], low, high)
    return clamped


def parse_required_float(pose, field_name):
    if field_name not in pose or pose[field_name] is None:
        raise ValueError("missing pose field: %s" % field_name)
    return float(pose[field_name])


def pose_to_target_angles(pose, limits, home_angles=None):
    home = home_angles or BASE_ANGLES
    left_raise = parse_required_float(pose, "left_arm_raise_angle")
    left_elbow = parse_required_float(pose, "left_elbow_angle")
    right_raise = parse_required_float(pose, "right_arm_raise_angle")
    right_elbow = parse_required_float(pose, "right_elbow_angle")

    left_elbow_bend = 180.0 - left_elbow
    right_elbow_bend = 180.0 - right_elbow

    left_shoulder_delta = map_range(left_raise, 0.0, 120.0, 0.0, 35.0)
    right_shoulder_delta = map_range(right_raise, 0.0, 120.0, 0.0, 35.0)
    left_elbow_delta = map_range(left_elbow_bend, 0.0, 120.0, 0.0, 35.0)
    right_elbow_delta = map_range(right_elbow_bend, 0.0, 120.0, 0.0, 35.0)

    target = {
        "left_shoulder": home["left_shoulder"] + left_shoulder_delta,
        "right_shoulder": home["right_shoulder"] - right_shoulder_delta,
        "left_elbow": home["left_elbow"] - left_elbow_delta,
        "right_elbow": home["right_elbow"] + right_elbow_delta,
    }

    return clamp_angles(target, limits)


def parse_joint_ids(text):
    values = [item.strip() for item in text.split(",") if item.strip()]
    if len(values) != 4:
        raise argparse.ArgumentTypeError("--joint-ids must contain four comma-separated IDs")
    ids = [int(value) for value in values]
    return dict((JOINT_NAMES[index], ids[index]) for index in range(4))


def parse_joint_limits(text):
    limits = JOINT_LIMITS.copy()
    if not text:
        return limits

    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        parts = item.split(":")
        if len(parts) != 3:
            raise argparse.ArgumentTypeError(
                "joint limit must look like left_shoulder:-100:20"
            )
        name, low, high = parts
        if name not in limits:
            raise argparse.ArgumentTypeError("unknown joint limit name: %s" % name)
        limits[name] = (float(low), float(high))
    return limits


def get_slot_type(msg, field_name):
    slots = getattr(msg, "__slots__", [])
    slot_types = getattr(msg, "_slot_types", [])
    if field_name in slots:
        index = slots.index(field_name)
        if index < len(slot_types):
            return slot_types[index]
    return ""


def set_message_field(msg, field_names, values):
    for field_name in field_names:
        if not hasattr(msg, field_name):
            continue
        slot_type = get_slot_type(msg, field_name)
        try:
            if slot_type.endswith("[]"):
                setattr(msg, field_name, list(values))
            else:
                setattr(msg, field_name, values[0])
            return field_name
        except Exception:
            continue
    return None


def joint_control_supports_joint_ids():
    msg = JointControlPoint()
    return any(
        hasattr(msg, field_name)
        for field_name in [
            "jointIdList",
            "jointIDList",
            "jointIds",
            "jointIDs",
            "idList",
            "ids",
            "jointId",
            "jointID",
        ]
    )


def build_joint_control_message(joint_ids, angles, control_id, time_from_start=None):
    msg = JointControlPoint()
    id_field = set_message_field(
        msg,
        ["jointIdList", "jointIDList", "jointIds", "jointIDs", "idList", "ids", "jointId", "jointID"],
        [int(value) for value in joint_ids],
    )
    angle_field = set_message_field(
        msg,
        ["angleList", "angles", "positionList", "positions", "jointPosition", "jointPositions", "data"],
        [float(value) for value in angles],
    )
    control_field = set_message_field(
        msg,
        ["mainControlID", "mainControlId", "main_control_id", "controlID", "controlId"],
        [int(control_id)],
    )
    if time_from_start is not None and hasattr(msg, "time_from_start"):
        msg.time_from_start = rospy.Duration.from_sec(float(time_from_start))

    if angle_field is None:
        raise RuntimeError(
            "Could not find an angle/position field in JointControlPoint fields: %s"
            % (getattr(msg, "__slots__", []),)
        )
    return msg, id_field, angle_field, control_field


def extract_control_id(response):
    for field_name in ["mainControlID", "mainControlId", "masterID", "masterId", "controlID", "id", "data"]:
        if hasattr(response, field_name):
            value = getattr(response, field_name)
            try:
                return int(value)
            except (TypeError, ValueError):
                continue

    slots = getattr(response, "__slots__", [])
    for slot in slots:
        value = getattr(response, slot)
        try:
            return int(value)
        except (TypeError, ValueError):
            continue
    return 0


def get_bodyhub_control_id(service_name):
    try:
        from bodyhub.srv import SrvTLSstring
    except Exception as err:
        rospy.logwarn("Could not import bodyhub.srv.SrvTLSstring: %s.", err)
        return 0

    try:
        rospy.wait_for_service(service_name, timeout=3.0)
        proxy = rospy.ServiceProxy(service_name, SrvTLSstring)
        response = proxy("get")
        return extract_control_id(response)
    except Exception as err:
        rospy.logwarn("Could not call %s: %s.", service_name, err)
        return 0


def get_bodyhub_status(status_service):
    try:
        from bodyhub.srv import SrvString
    except Exception as err:
        rospy.logwarn("Could not import bodyhub.srv.SrvString: %s.", err)
        return None

    try:
        rospy.wait_for_service(status_service, timeout=3.0)
        proxy = rospy.ServiceProxy(status_service, SrvString)
        response = proxy("get")
        return response.data
    except Exception as err:
        rospy.logwarn("Could not call %s: %s.", status_service, err)
        return None


def set_bodyhub_status(state_service, control_id, state):
    try:
        from bodyhub.srv import SrvState
    except Exception as err:
        rospy.logwarn("Could not import bodyhub.srv.SrvState: %s.", err)
        return None

    try:
        rospy.wait_for_service(state_service, timeout=3.0)
        proxy = rospy.ServiceProxy(state_service, SrvState)
        response = proxy(int(control_id), state)
        return response.stateRes
    except Exception as err:
        rospy.logwarn("Could not call %s %s: %s.", state_service, state, err)
        return None


def prepare_bodyhub(args, control_id):
    current_id = get_bodyhub_control_id(args.service)
    status = get_bodyhub_status(args.status_service)
    rospy.loginfo("BodyHub before prepare: status=%s currentControlID=%s", status, current_id)

    if current_id not in (0, int(control_id)):
        rospy.logwarn(
            "BodyHub appears busy with control id %s; requested id is %s.",
            current_id,
            control_id,
        )

    if status in ("ready", "running", "pause"):
        return True

    if status == "preReady":
        result = set_bodyhub_status(args.state_service, control_id, "setStatus")
        rospy.loginfo("StateJump setStatus result=%s", result)
        return result in (22, 23)

    rospy.logwarn("BodyHub status %s is not prepared automatically.", status)
    return False


def get_main_control_id(args):
    if args.control_id is not None:
        return int(args.control_id)

    control_id = get_bodyhub_control_id(args.service)
    if control_id == 0 and args.default_control_id:
        return int(args.default_control_id)
    return control_id


class ServoUpperBodyController(object):
    def __init__(self, args, joint_ids, joint_limits):
        self.args = args
        self.joint_ids = joint_ids
        self.joint_limits = joint_limits
        self.enabled_joint_names = ARM_JOINTS[args.enabled_arms]
        self.state = TargetState()
        self.current_angles = BASE_ANGLES.copy()
        self.home_angles = BASE_ANGLES.copy()
        self.running = True
        self.last_log_time = 0.0
        self.last_stale_warning_time = 0.0
        self.last_bodyhub_check_time = 0.0
        self.last_bodyhub_warning_time = 0.0
        self.last_bodyhub_signature = None
        self.in_valid_segment = False
        self.pre_action_home_until = 0.0
        self.shutdown_started = False
        self.control_id = get_main_control_id(args)
        if args.prepare_bodyhub:
            prepare_bodyhub(args, self.control_id)
        self.publisher = rospy.Publisher(args.joint_topic, JointControlPoint, queue_size=10)
        self.supports_joint_ids = joint_control_supports_joint_ids()
        self.full_position_frame = None

        if not self.supports_joint_ids:
            self.full_position_frame = self.load_initial_full_position_frame()
            self.initialize_current_angles_from_frame(self.full_position_frame)

    def load_initial_full_position_frame(self):
        try:
            msg = rospy.wait_for_message(
                self.args.servo_position_topic,
                ServoPositionAngle,
                timeout=2.0,
            )
            frame = [float(value) for value in msg.angle]
            if len(frame) >= max(self.joint_ids.values()):
                rospy.loginfo(
                    "Using current servo frame from %s, length=%d",
                    self.args.servo_position_topic,
                    len(frame),
                )
                return frame
            rospy.logwarn(
                "Servo position frame from %s is too short: length=%d",
                self.args.servo_position_topic,
                len(frame),
            )
        except Exception as err:
            rospy.logwarn("Could not read initial servo positions: %s", err)

        rospy.logwarn("Falling back to static BASE_FRAME for full-position publish mode.")
        return BASE_FRAME[:]

    def initialize_current_angles_from_frame(self, frame):
        for name in JOINT_NAMES:
            index = self.joint_ids[name] - 1
            if 0 <= index < len(frame):
                low, high = self.joint_limits[name]
                self.current_angles[name] = clamp(float(frame[index]), low, high)

    def pose_callback(self, msg):
        try:
            pose = json.loads(msg.data)
        except Exception as err:
            rospy.logwarn("Invalid pose JSON: %s", err)
            self.state.update(False, 0.0, {}, self.home_angles)
            return

        visible = bool(pose.get("visible", False))
        try:
            confidence = float(pose.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0

        if not visible or confidence < self.args.confidence_threshold:
            self.state.update(visible, confidence, pose, self.home_angles)
            return

        try:
            target_angles = pose_to_target_angles(
                pose,
                self.joint_limits,
                self.home_angles,
            )
        except Exception as err:
            rospy.logwarn("Could not map pose to target angles: %s", err)
            self.state.update(False, confidence, pose, self.home_angles)
            return

        self.state.update(visible, confidence, pose, target_angles)

    def get_control_target(self):
        snapshot = self.state.snapshot()
        now = time.time()
        stale = (
            snapshot["timestamp"] <= 0.0
            or now - snapshot["timestamp"] > self.args.stale_timeout
        )
        valid = (
            (not stale)
            and snapshot["visible"]
            and snapshot["confidence"] >= self.args.confidence_threshold
        )

        if stale:
            if now - self.last_stale_warning_time > 1.0:
                rospy.logwarn(
                    "pose stale, returning to base pose "
                    "(message_age=%.2fs)",
                    now - snapshot["timestamp"] if snapshot["timestamp"] > 0.0 else -1.0,
                )
                self.last_stale_warning_time = now
            return self.home_angles.copy(), False, snapshot

        if not valid:
            return self.home_angles.copy(), False, snapshot

        return snapshot["target_angles"].copy(), True, snapshot

    def apply_action_home_policy(self, target, valid):
        now = time.time()

        if valid:
            if not self.in_valid_segment:
                self.in_valid_segment = True
                self.pre_action_home_until = now + self.args.pre_action_home_duration
                rospy.loginfo(
                    "New valid pose segment; holding home for %.2fs before following.",
                    self.args.pre_action_home_duration,
                )

            if (
                now < self.pre_action_home_until
                or not self.is_home_reached()
            ):
                return self.home_angles.copy(), False

            return target, True

        if self.in_valid_segment:
            rospy.loginfo("Pose segment ended; returning to home.")
            self.in_valid_segment = False
            self.pre_action_home_until = 0.0

        return self.home_angles.copy(), False

    def is_home_reached(self):
        for name in self.enabled_joint_names:
            if abs(self.current_angles[name] - self.home_angles[name]) > self.args.home_tolerance_deg:
                return False
        return True

    def smooth_current_angles(self, target):
        for name in self.enabled_joint_names:
            filtered = low_pass(self.current_angles[name], target[name], self.args.alpha)
            limited = slew_limit(self.current_angles[name], filtered, self.args.max_step_deg)
            low, high = self.joint_limits[name]
            self.current_angles[name] = clamp(limited, low, high)

    def publish_current_angles(self):
        joint_id_list = [self.joint_ids[name] for name in self.enabled_joint_names]
        time_from_start = 1.0 / max(1.0, self.args.hz)

        if self.supports_joint_ids:
            angle_list = [self.current_angles[name] for name in self.enabled_joint_names]
        else:
            angle_list = self.full_position_frame[:]
            for name in self.enabled_joint_names:
                index = self.joint_ids[name] - 1
                if not 0 <= index < len(angle_list):
                    raise RuntimeError(
                        "Joint ID %s for %s is outside full frame length %s"
                        % (self.joint_ids[name], name, len(angle_list))
                    )
                angle_list[index] = self.current_angles[name]

        try:
            msg, _, _, _ = build_joint_control_message(
                joint_id_list,
                angle_list,
                self.control_id,
                time_from_start=time_from_start,
            )
            self.publisher.publish(msg)
        except Exception:
            self.publisher.publish(positions=angle_list, mainControlID=self.control_id)

    def keep_bodyhub_ready(self):
        if not self.args.prepare_bodyhub:
            return

        now = time.time()
        if now - self.last_bodyhub_check_time < self.args.bodyhub_check_interval:
            return
        self.last_bodyhub_check_time = now

        current_id = get_bodyhub_control_id(self.args.service)
        status = get_bodyhub_status(self.args.status_service)
        signature = (status, current_id)
        if signature != self.last_bodyhub_signature:
            rospy.loginfo("BodyHub status=%s currentControlID=%s", status, current_id)
            self.last_bodyhub_signature = signature

        if status == "preReady" or current_id not in (int(self.control_id),):
            result = set_bodyhub_status(
                self.args.state_service,
                self.control_id,
                "setStatus",
            )
            rospy.logwarn(
                "Re-requested BodyHub control: status=%s currentControlID=%s "
                "requestedID=%s stateRes=%s",
                status,
                current_id,
                self.control_id,
                result,
            )
            return

        if status not in ("ready", "running", "pause"):
            if now - self.last_bodyhub_warning_time > 2.0:
                rospy.logwarn(
                    "Unexpected BodyHub status=%s currentControlID=%s",
                    status,
                    current_id,
                )
                self.last_bodyhub_warning_time = now

    def log_status(self, valid, snapshot):
        now = time.time()
        if now - self.last_log_time < 0.5:
            return
        self.last_log_time = now
        pose = snapshot.get("pose", {})

        rospy.loginfo(
            "servo controller running hz=%.1f\n"
            "target valid=%s conf=%.2f\n"
            "pose angles: left_raise=%s left_elbow=%s right_raise=%s right_elbow=%s\n"
            "current angles: LS=%.1f LE=%.1f RS=%.1f RE=%.1f\n"
            "publish topic: %s joint_ids=%s enabled_arms=%s",
            self.args.hz,
            valid,
            snapshot["confidence"],
            pose.get("left_arm_raise_angle"),
            pose.get("left_elbow_angle"),
            pose.get("right_arm_raise_angle"),
            pose.get("right_elbow_angle"),
            self.current_angles["left_shoulder"],
            self.current_angles["left_elbow"],
            self.current_angles["right_shoulder"],
            self.current_angles["right_elbow"],
            self.args.joint_topic,
            [self.joint_ids[name] for name in self.enabled_joint_names],
            self.args.enabled_arms,
        )

    def control_loop(self):
        rate = rospy.Rate(self.args.hz)
        while not rospy.is_shutdown() and self.running:
            target, valid, snapshot = self.get_control_target()
            target, valid = self.apply_action_home_policy(target, valid)
            self.smooth_current_angles(target)
            self.keep_bodyhub_ready()
            self.publish_current_angles()
            self.log_status(valid, snapshot)
            rate.sleep()

    def shutdown(self):
        if self.shutdown_started:
            self.running = False
            return
        self.shutdown_started = True
        self.running = False
        if not hasattr(self, "publisher"):
            return

        deadline = time.time() + self.args.shutdown_home_duration
        interval = 1.0 / max(1.0, self.args.hz)
        while time.time() < deadline:
            self.smooth_current_angles(self.home_angles)
            self.keep_bodyhub_ready()
            self.publish_current_angles()
            time.sleep(interval)


def build_arg_parser():
    default_ids = ",".join(str(DEFAULT_JOINT_IDS[name]) for name in JOINT_NAMES)
    parser = argparse.ArgumentParser(description="Week 4 high-rate Roban upper-body servo controller.")
    parser.add_argument("--input-topic", default=DEFAULT_INPUT_TOPIC, help="Pose JSON input topic.")
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC, help="JointControlPoint output topic.")
    parser.add_argument(
        "--servo-position-topic",
        default=DEFAULT_SERVO_POSITION_TOPIC,
        help="Servo position topic used to preserve non-upper-body joints in full-frame mode.",
    )
    parser.add_argument(
        "--joint-ids",
        type=parse_joint_ids,
        default=parse_joint_ids(default_ids),
        help="Four IDs in left_shoulder,left_elbow,right_shoulder,right_elbow order.",
    )
    parser.add_argument("--hz", type=float, default=DEFAULT_CONTROL_HZ, help="Servo publish frequency.")
    parser.add_argument(
        "--enabled-arms",
        choices=["left", "right", "both"],
        default="both",
        help="Which arm targets to publish. Disabled arms keep their startup servo angles.",
    )
    parser.add_argument("--confidence-threshold", type=float, default=0.85, help="Minimum pose confidence.")
    parser.add_argument("--stale-timeout", type=float, default=0.6, help="Seconds before pose data is stale.")
    parser.add_argument("--pre-action-home-duration", type=float, default=0.3, help="Seconds to hold home before following a fresh valid pose segment.")
    parser.add_argument("--shutdown-home-duration", type=float, default=1.5, help="Seconds to publish home during controller shutdown.")
    parser.add_argument("--home-tolerance-deg", type=float, default=2.0, help="Max joint error allowed before a new pose segment can start.")
    parser.add_argument("--alpha", type=float, default=0.25, help="Low-pass filter coefficient.")
    parser.add_argument("--max-step-deg", type=float, default=DEFAULT_MAX_STEP_DEG, help="Max per-cycle angle step in degrees.")
    parser.add_argument(
        "--joint-limits",
        type=parse_joint_limits,
        default=JOINT_LIMITS.copy(),
        help="Optional limits like left_shoulder:-100:20,left_elbow:-70:30.",
    )
    parser.add_argument("--service", default=DEFAULT_SERVICE, help="GetMasterID service name.")
    parser.add_argument("--status-service", default="/MediumSize/BodyHub/GetStatus", help="BodyHub status service name.")
    parser.add_argument("--state-service", default="/MediumSize/BodyHub/StateJump", help="BodyHub state transition service name.")
    parser.add_argument("--control-id", type=int, default=None, help="Override mainControlID instead of calling service.")
    parser.add_argument("--default-control-id", type=int, default=DEFAULT_CONTROL_ID, help="Control ID used when BodyHub reports 0.")
    parser.add_argument("--bodyhub-check-interval", type=float, default=1.0, help="Seconds between runtime BodyHub status checks.")
    parser.add_argument("--prepare-bodyhub", action="store_true", default=True, help="Set BodyHub to ready when it is preReady.")
    parser.add_argument("--no-prepare-bodyhub", action="store_false", dest="prepare_bodyhub", help="Do not change BodyHub state at startup.")
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    args.alpha = clamp(args.alpha, 0.0, 1.0)
    args.max_step_deg = max(0.0, args.max_step_deg)
    args.hz = max(1.0, args.hz)
    args.bodyhub_check_interval = max(0.2, args.bodyhub_check_interval)
    args.pre_action_home_duration = max(0.0, args.pre_action_home_duration)
    args.shutdown_home_duration = max(0.0, args.shutdown_home_duration)
    args.home_tolerance_deg = max(0.0, args.home_tolerance_deg)

    rospy.init_node("week4_servo_upper_body_controller", anonymous=True)

    controller = ServoUpperBodyController(args, args.joint_ids, args.joint_limits)
    rospy.Subscriber(args.input_topic, String, controller.pose_callback, queue_size=1)
    rospy.on_shutdown(controller.shutdown)

    probe_msg = JointControlPoint()
    rospy.loginfo("Week4 servo upper-body controller started.")
    rospy.loginfo("Subscribing topic: %s", args.input_topic)
    rospy.loginfo("Publishing topic: %s", args.joint_topic)
    rospy.loginfo("Enabled arms: %s", args.enabled_arms)
    rospy.loginfo("JointControlPoint fields: %s", getattr(probe_msg, "__slots__", []))
    rospy.loginfo("Joint ID field supported: %s", controller.supports_joint_ids)
    rospy.loginfo("mainControlID=%s", controller.control_id)

    thread = threading.Thread(target=controller.control_loop)
    thread.daemon = True
    thread.start()
    rospy.spin()
    controller.shutdown()
    thread.join(1.0)


if __name__ == "__main__":
    main()
