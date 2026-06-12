#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import json
import threading
import time

import rospy
from bodyhub.msg import JointControlPoint
from std_msgs.msg import String


JOINT_NAMES = ["left_shoulder", "left_elbow", "right_shoulder", "right_elbow"]

BASE_ANGLES = {
    "left_shoulder": -61.0,
    "left_elbow": -18.0,
    "right_shoulder": 61.0,
    "right_elbow": 18.0,
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
DEFAULT_JOINT_TOPIC = "/MediumSize/BodyHub/JointPosition"
DEFAULT_SERVICE = "/MediumSize/BodyHub/GetMasterID"
DEFAULT_CONTROL_HZ = 100.0
DEFAULT_MAX_STEP_DEG = 0.6


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


def pose_to_target_angles(pose, limits):
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
        "left_shoulder": BASE_ANGLES["left_shoulder"] + left_shoulder_delta,
        "right_shoulder": BASE_ANGLES["right_shoulder"] - right_shoulder_delta,
        "left_elbow": BASE_ANGLES["left_elbow"] - left_elbow_delta,
        "right_elbow": BASE_ANGLES["right_elbow"] + right_elbow_delta,
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


def build_joint_control_message(joint_ids, angles, control_id):
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


def get_main_control_id(service_name, explicit_control_id=None):
    if explicit_control_id is not None:
        return int(explicit_control_id)

    try:
        from bodyhub.srv import GetMasterID
    except Exception as err:
        rospy.logwarn("Could not import bodyhub.srv.GetMasterID: %s. Using control id 0.", err)
        return 0

    try:
        rospy.wait_for_service(service_name, timeout=3.0)
        proxy = rospy.ServiceProxy(service_name, GetMasterID)
        response = proxy()
        return extract_control_id(response)
    except Exception as err:
        rospy.logwarn("Could not call %s: %s. Using control id 0.", service_name, err)
        return 0


class ServoUpperBodyController(object):
    def __init__(self, args, joint_ids, joint_limits):
        self.args = args
        self.joint_ids = joint_ids
        self.joint_limits = joint_limits
        self.state = TargetState()
        self.current_angles = BASE_ANGLES.copy()
        self.running = True
        self.last_log_time = 0.0
        self.last_stale_warning_time = 0.0
        self.control_id = get_main_control_id(args.service, args.control_id)
        self.publisher = rospy.Publisher(args.joint_topic, JointControlPoint, queue_size=10)

    def pose_callback(self, msg):
        try:
            pose = json.loads(msg.data)
        except Exception as err:
            rospy.logwarn("Invalid pose JSON: %s", err)
            self.state.update(False, 0.0, {}, BASE_ANGLES)
            return

        visible = bool(pose.get("visible", False))
        try:
            confidence = float(pose.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0

        if not visible or confidence < self.args.confidence_threshold:
            self.state.update(visible, confidence, pose, BASE_ANGLES)
            return

        try:
            target_angles = pose_to_target_angles(pose, self.joint_limits)
        except Exception as err:
            rospy.logwarn("Could not map pose to target angles: %s", err)
            self.state.update(False, confidence, pose, BASE_ANGLES)
            return

        self.state.update(visible, confidence, pose, target_angles)

    def get_control_target(self):
        snapshot = self.state.snapshot()
        now = time.time()
        stale = snapshot["timestamp"] <= 0.0 or now - snapshot["timestamp"] > self.args.stale_timeout
        valid = (
            (not stale)
            and snapshot["visible"]
            and snapshot["confidence"] >= self.args.confidence_threshold
        )

        if stale:
            if now - self.last_stale_warning_time > 1.0:
                rospy.logwarn("pose stale, returning to base pose")
                self.last_stale_warning_time = now
            return BASE_ANGLES.copy(), False, snapshot

        if not valid:
            return BASE_ANGLES.copy(), False, snapshot

        return snapshot["target_angles"].copy(), True, snapshot

    def smooth_current_angles(self, target):
        for name in JOINT_NAMES:
            filtered = low_pass(self.current_angles[name], target[name], self.args.alpha)
            limited = slew_limit(self.current_angles[name], filtered, self.args.max_step_deg)
            low, high = self.joint_limits[name]
            self.current_angles[name] = clamp(limited, low, high)

    def publish_current_angles(self):
        joint_id_list = [self.joint_ids[name] for name in JOINT_NAMES]
        angle_list = [self.current_angles[name] for name in JOINT_NAMES]

        try:
            msg, _, _, _ = build_joint_control_message(joint_id_list, angle_list, self.control_id)
            self.publisher.publish(msg)
        except Exception:
            self.publisher.publish(positions=angle_list, mainControlID=self.control_id)

    def log_status(self, valid, snapshot):
        now = time.time()
        if now - self.last_log_time < 0.5:
            return
        self.last_log_time = now

        rospy.loginfo(
            "servo controller running hz=%.1f\n"
            "target valid=%s conf=%.2f\n"
            "current angles: LS=%.1f LE=%.1f RS=%.1f RE=%.1f\n"
            "publish topic: %s joint_ids=%s",
            self.args.hz,
            valid,
            snapshot["confidence"],
            self.current_angles["left_shoulder"],
            self.current_angles["left_elbow"],
            self.current_angles["right_shoulder"],
            self.current_angles["right_elbow"],
            self.args.joint_topic,
            [self.joint_ids[name] for name in JOINT_NAMES],
        )

    def control_loop(self):
        rate = rospy.Rate(self.args.hz)
        while not rospy.is_shutdown() and self.running:
            target, valid, snapshot = self.get_control_target()
            self.smooth_current_angles(target)
            self.publish_current_angles()
            self.log_status(valid, snapshot)
            rate.sleep()

    def shutdown(self):
        self.running = False


def build_arg_parser():
    default_ids = ",".join(str(DEFAULT_JOINT_IDS[name]) for name in JOINT_NAMES)
    parser = argparse.ArgumentParser(description="Week 4 high-rate Roban upper-body servo controller.")
    parser.add_argument("--input-topic", default=DEFAULT_INPUT_TOPIC, help="Pose JSON input topic.")
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC, help="JointControlPoint output topic.")
    parser.add_argument(
        "--joint-ids",
        type=parse_joint_ids,
        default=parse_joint_ids(default_ids),
        help="Four IDs in left_shoulder,left_elbow,right_shoulder,right_elbow order.",
    )
    parser.add_argument("--hz", type=float, default=DEFAULT_CONTROL_HZ, help="Servo publish frequency.")
    parser.add_argument("--confidence-threshold", type=float, default=0.85, help="Minimum pose confidence.")
    parser.add_argument("--stale-timeout", type=float, default=0.6, help="Seconds before pose data is stale.")
    parser.add_argument("--alpha", type=float, default=0.25, help="Low-pass filter coefficient.")
    parser.add_argument("--max-step-deg", type=float, default=DEFAULT_MAX_STEP_DEG, help="Max per-cycle angle step in degrees.")
    parser.add_argument(
        "--joint-limits",
        type=parse_joint_limits,
        default=JOINT_LIMITS.copy(),
        help="Optional limits like left_shoulder:-100:20,left_elbow:-70:30.",
    )
    parser.add_argument("--service", default=DEFAULT_SERVICE, help="GetMasterID service name.")
    parser.add_argument("--control-id", type=int, default=None, help="Override mainControlID instead of calling service.")
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    args.alpha = clamp(args.alpha, 0.0, 1.0)
    args.max_step_deg = max(0.0, args.max_step_deg)
    args.hz = max(1.0, args.hz)

    rospy.init_node("week4_servo_upper_body_controller", anonymous=True)

    controller = ServoUpperBodyController(args, args.joint_ids, args.joint_limits)
    rospy.Subscriber(args.input_topic, String, controller.pose_callback, queue_size=1)
    rospy.on_shutdown(controller.shutdown)

    probe_msg = JointControlPoint()
    rospy.loginfo("Week4 servo upper-body controller started.")
    rospy.loginfo("Subscribing topic: %s", args.input_topic)
    rospy.loginfo("Publishing topic: %s", args.joint_topic)
    rospy.loginfo("JointControlPoint fields: %s", getattr(probe_msg, "__slots__", []))
    rospy.loginfo("mainControlID=%s", controller.control_id)

    thread = threading.Thread(target=controller.control_loop)
    thread.daemon = True
    thread.start()
    rospy.spin()
    controller.shutdown()
    thread.join(1.0)


if __name__ == "__main__":
    main()
