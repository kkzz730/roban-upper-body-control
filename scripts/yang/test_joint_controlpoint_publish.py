#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import math
import time

import rospy
from bodyhub.msg import JointControlPoint
from bodyhub.msg import ServoPositionAngle


DEFAULT_TOPIC = "/MediumSize/BodyHub/MotoPosition"
DEFAULT_SERVO_POSITION_TOPIC = "/MediumSize/BodyHub/ServoPositions"
DEFAULT_SERVICE = "/MediumSize/BodyHub/GetMasterID"
MAX_DEFAULT_AMPLITUDE = 8.0
BASE_FRAME = [0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0]


def clamp(value, low, high):
    return max(low, min(high, value))


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

    if isinstance(response, (int, float)):
        return int(response)

    slots = getattr(response, "__slots__", [])
    for slot in slots:
        value = getattr(response, slot)
        try:
            return int(value)
        except (TypeError, ValueError):
            continue

    return 0


def load_initial_full_position_frame(topic, joint_id):
    try:
        msg = rospy.wait_for_message(topic, ServoPositionAngle, timeout=2.0)
        frame = [float(value) for value in msg.angle]
        if len(frame) >= joint_id:
            rospy.loginfo("Using current servo frame from %s, length=%d", topic, len(frame))
            return frame
        rospy.logwarn("Servo position frame from %s is too short: length=%d", topic, len(frame))
    except Exception as err:
        rospy.logwarn("Could not read initial servo positions: %s", err)

    rospy.logwarn("Falling back to static BASE_FRAME for full-position publish mode.")
    return BASE_FRAME[:]


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


def build_angle_payload(joint_id, angle, supports_joint_ids, full_position_frame):
    if supports_joint_ids:
        return [joint_id], [angle]

    positions = full_position_frame[:]
    index = joint_id - 1
    if not 0 <= index < len(positions):
        raise RuntimeError(
            "Joint ID %s is outside full frame length %s" % (joint_id, len(positions))
        )
    positions[index] = angle
    return [], positions


def publish_joint(pub, joint_id, angle, control_id, supports_joint_ids, full_position_frame, hz):
    joint_ids, angles = build_angle_payload(joint_id, angle, supports_joint_ids, full_position_frame)
    try:
        msg, _, _, _ = build_joint_control_message(
            joint_ids,
            angles,
            control_id,
            time_from_start=1.0 / max(1.0, hz),
        )
        pub.publish(msg)
    except Exception:
        pub.publish(positions=angles, mainControlID=control_id)


def main():
    parser = argparse.ArgumentParser(description="Publish a small JointControlPoint test motion.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="JointControlPoint topic.")
    parser.add_argument("--joint-id", type=int, default=17, help="Single joint ID to test.")
    parser.add_argument("--base-angle", type=float, default=61.0, help="Base angle in degrees.")
    parser.add_argument("--amplitude", type=float, default=5.0, help="Sine amplitude in degrees, clamped to +/-8.")
    parser.add_argument("--hz", type=float, default=20.0, help="Publish frequency.")
    parser.add_argument("--duration", type=float, default=5.0, help="Motion duration in seconds.")
    parser.add_argument(
        "--servo-position-topic",
        default=DEFAULT_SERVO_POSITION_TOPIC,
        help="Servo position topic used to preserve non-tested joints in full-frame mode.",
    )
    parser.add_argument("--service", default=DEFAULT_SERVICE, help="GetMasterID service name.")
    parser.add_argument("--control-id", type=int, default=None, help="Override mainControlID instead of calling service.")
    args = parser.parse_args()

    rospy.init_node("test_joint_controlpoint_publish", anonymous=True)

    amplitude = clamp(abs(args.amplitude), 0.0, MAX_DEFAULT_AMPLITUDE)
    if amplitude != abs(args.amplitude):
        rospy.logwarn("Amplitude %.1f is larger than safe default %.1f. Clamped.", args.amplitude, MAX_DEFAULT_AMPLITUDE)

    control_id = get_main_control_id(args.service, args.control_id)
    pub = rospy.Publisher(args.topic, JointControlPoint, queue_size=10)
    supports_joint_ids = joint_control_supports_joint_ids()
    full_position_frame = None
    if not supports_joint_ids:
        full_position_frame = load_initial_full_position_frame(args.servo_position_topic, args.joint_id)

    probe_msg = JointControlPoint()
    rospy.loginfo("JointControlPoint fields: %s", getattr(probe_msg, "__slots__", []))
    rospy.loginfo("Joint ID field supported: %s", supports_joint_ids)
    rospy.sleep(0.5)

    rate = rospy.Rate(args.hz)
    start = time.time()
    period = max(2.0, args.duration)

    while not rospy.is_shutdown() and time.time() - start < args.duration:
        elapsed = time.time() - start
        angle = args.base_angle + amplitude * math.sin(2.0 * math.pi * elapsed / period)
        publish_joint(pub, args.joint_id, angle, control_id, supports_joint_ids, full_position_frame, args.hz)
        print(
            "publish joint_id=%s angle=%.1f mainControlID=%s"
            % (args.joint_id, angle, control_id)
        )
        rate.sleep()

    current = args.base_angle + amplitude * math.sin(2.0 * math.pi * min(args.duration, time.time() - start) / period)
    steps = max(1, int(args.hz))
    for index in range(steps):
        if rospy.is_shutdown():
            break
        ratio = float(index + 1) / float(steps)
        angle = current + ratio * (args.base_angle - current)
        publish_joint(pub, args.joint_id, angle, control_id, supports_joint_ids, full_position_frame, args.hz)
        print(
            "publish joint_id=%s angle=%.1f mainControlID=%s"
            % (args.joint_id, angle, control_id)
        )
        rate.sleep()


if __name__ == "__main__":
    main()
