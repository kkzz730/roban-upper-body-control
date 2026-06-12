#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import json

import rospy


DEFAULT_TOPIC = "/MediumSize/BodyHub/ServoPositions"


def is_number(value):
    return isinstance(value, (int, float))


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def get_field(msg, names):
    for name in names:
        if hasattr(msg, name):
            return getattr(msg, name)
    return None


def parse_json_positions(text):
    try:
        data = json.loads(text)
    except Exception:
        return None

    if isinstance(data, dict):
        ids = data.get("servoIdList") or data.get("idList") or data.get("ids")
        values = (
            data.get("angleList")
            or data.get("positionList")
            or data.get("positions")
            or data.get("data")
        )
        if isinstance(ids, list) and isinstance(values, list):
            return dict((int(ids[i]), float(values[i])) for i in range(min(len(ids), len(values))))
        if isinstance(values, list):
            return dict((i + 1, float(value)) for i, value in enumerate(values))

    if isinstance(data, list):
        return dict((i + 1, float(value)) for i, value in enumerate(data))

    return None


def parse_object_list(items):
    positions = {}
    for index, item in enumerate(items):
        if is_number(item):
            positions[index + 1] = float(item)
            continue

        servo_id = get_field(item, ["id", "ID", "servoId", "servoID", "jointId", "jointID"])
        angle = get_field(item, ["angle", "position", "pos", "value", "data"])
        if servo_id is not None and angle is not None:
            angle_value = to_float(angle)
            if angle_value is not None:
                positions[int(servo_id)] = angle_value

    return positions or None


def extract_positions(msg):
    if hasattr(msg, "data") and isinstance(msg.data, str):
        parsed = parse_json_positions(msg.data)
        if parsed:
            return parsed

    ids = get_field(msg, ["servoIdList", "servoIDList", "idList", "ids", "jointIdList", "jointIDList"])
    values = get_field(
        msg,
        [
            "angleList",
            "positionList",
            "positions",
            "position",
            "servoPositionList",
            "servoPositions",
            "data",
        ],
    )

    if ids is not None and values is not None:
        return dict((int(ids[i]), float(values[i])) for i in range(min(len(ids), len(values))))

    if values is not None:
        if isinstance(values, str):
            parsed = parse_json_positions(values)
            if parsed:
                return parsed
        try:
            return parse_object_list(list(values))
        except TypeError:
            pass

    slots = getattr(msg, "__slots__", [])
    for slot in slots:
        value = getattr(msg, slot)
        if isinstance(value, str):
            parsed = parse_json_positions(value)
            if parsed:
                return parsed
        try:
            parsed = parse_object_list(list(value))
            if parsed:
                return parsed
        except TypeError:
            continue

    return None


class ServoMonitor(object):
    def __init__(self, topic, threshold):
        self.topic = topic
        self.threshold = threshold
        self.previous = None

    def callback(self, msg):
        positions = extract_positions(msg)
        if not positions:
            rospy.logwarn("Could not extract servo positions from message type %s", type(msg).__name__)
            return

        summary = " ".join(
            "id=%s:%.1f" % (servo_id, positions[servo_id])
            for servo_id in sorted(positions.keys())
        )
        print("servo positions:", summary)

        if self.previous is not None:
            for servo_id in sorted(positions.keys()):
                if servo_id not in self.previous:
                    continue
                old = self.previous[servo_id]
                new = positions[servo_id]
                delta = new - old
                if abs(delta) > self.threshold:
                    print(
                        "changed servo: id=%s old=%.1f new=%.1f delta=%.1f"
                        % (servo_id, old, new, delta)
                    )

        self.previous = positions


def resolve_topic_class(topic):
    import rostopic

    msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=True)
    if msg_class is None:
        raise RuntimeError("Could not resolve message type for topic %s" % topic)
    return msg_class, real_topic or topic


def main():
    parser = argparse.ArgumentParser(description="Monitor Roban servo position deltas.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Servo position topic to subscribe.")
    parser.add_argument(
        "--threshold",
        type=float,
        default=1.0,
        help="Only print changed servo IDs whose absolute delta is greater than this many degrees.",
    )
    args = parser.parse_args()

    rospy.init_node("monitor_servo_positions", anonymous=True)
    msg_class, real_topic = resolve_topic_class(args.topic)

    monitor = ServoMonitor(real_topic, args.threshold)
    rospy.Subscriber(real_topic, msg_class, monitor.callback, queue_size=1)

    print("Monitoring servo positions on %s" % real_topic)
    print("Message type: %s" % msg_class)
    print("Change threshold: %.1f deg" % args.threshold)
    rospy.spin()


if __name__ == "__main__":
    main()
