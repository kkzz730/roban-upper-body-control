#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import math
import time

import rospy
from bodyhub.msg import JointControlPoint


DEFAULT_TOPIC = "/MediumSize/BodyHub/JointPosition"
DEFAULT_SERVICE = "/MediumSize/BodyHub/GetMasterID"
MAX_DEFAULT_AMPLITUDE = 8.0


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


def publish_joint(pub, joint_id, angle, control_id):
    try:
        msg, _, _, _ = build_joint_control_message([joint_id], [angle], control_id)
        pub.publish(msg)
    except Exception:
        pub.publish(positions=[angle], mainControlID=control_id)


def main():
    parser = argparse.ArgumentParser(description="Publish a small JointControlPoint test motion.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="JointControlPoint topic.")
    parser.add_argument("--joint-id", type=int, default=17, help="Single joint ID to test.")
    parser.add_argument("--base-angle", type=float, default=61.0, help="Base angle in degrees.")
    parser.add_argument("--amplitude", type=float, default=5.0, help="Sine amplitude in degrees, clamped to +/-8.")
    parser.add_argument("--hz", type=float, default=20.0, help="Publish frequency.")
    parser.add_argument("--duration", type=float, default=5.0, help="Motion duration in seconds.")
    parser.add_argument("--service", default=DEFAULT_SERVICE, help="GetMasterID service name.")
    parser.add_argument("--control-id", type=int, default=None, help="Override mainControlID instead of calling service.")
    args = parser.parse_args()

    rospy.init_node("test_joint_controlpoint_publish", anonymous=True)

    amplitude = clamp(abs(args.amplitude), 0.0, MAX_DEFAULT_AMPLITUDE)
    if amplitude != abs(args.amplitude):
        rospy.logwarn("Amplitude %.1f is larger than safe default %.1f. Clamped.", args.amplitude, MAX_DEFAULT_AMPLITUDE)

    control_id = get_main_control_id(args.service, args.control_id)
    pub = rospy.Publisher(args.topic, JointControlPoint, queue_size=10)

    probe_msg = JointControlPoint()
    rospy.loginfo("JointControlPoint fields: %s", getattr(probe_msg, "__slots__", []))
    rospy.sleep(0.5)

    rate = rospy.Rate(args.hz)
    start = time.time()
    period = max(2.0, args.duration)

    while not rospy.is_shutdown() and time.time() - start < args.duration:
        elapsed = time.time() - start
        angle = args.base_angle + amplitude * math.sin(2.0 * math.pi * elapsed / period)
        publish_joint(pub, args.joint_id, angle, control_id)
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
        publish_joint(pub, args.joint_id, angle, control_id)
        print(
            "publish joint_id=%s angle=%.1f mainControlID=%s"
            % (args.joint_id, angle, control_id)
        )
        rate.sleep()


if __name__ == "__main__":
    main()
