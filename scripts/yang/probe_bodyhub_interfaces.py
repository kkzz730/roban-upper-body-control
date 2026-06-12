#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import os
import subprocess


DEFAULT_OUTPUT = "logs/week4/bodyhub_interface_probe.txt"

KEYWORDS = ["BodyHub", "Joint", "Position", "Servo"]
CONTROL_TOPIC_HINTS = [
    "/MediumSize/BodyHub/HeadPosition",
    "/MediumSize/BodyHub/BodyPosition",
    "/MediumSize/BodyHub/JointPosition",
    "/MediumSize/BodyHub/JointControl",
    "/MediumSize/BodyHub/ServoPosition",
    "/MediumSize/BodyHub/ServoPositions",
]


def run_command(command):
    try:
        output = subprocess.check_output(
            command,
            shell=True,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
        )
        return output.rstrip()
    except subprocess.CalledProcessError as err:
        if "| grep" in command and err.returncode == 1 and not err.output:
            return ""
        return err.output.rstrip() or "command failed with exit code %s" % err.returncode


def append_section(lines, title, body):
    lines.append("")
    lines.append("===== %s =====" % title)
    if body:
        lines.append(body)
    else:
        lines.append("(no output)")


def get_topic_list():
    text = run_command("rostopic list")
    if text.startswith("command failed"):
        return []
    return [line.strip() for line in text.splitlines() if line.strip()]


def collect_candidate_topics(topics):
    candidates = []
    seen = set()

    for hint in CONTROL_TOPIC_HINTS:
        if hint in topics and hint not in seen:
            candidates.append(hint)
            seen.add(hint)

    lower_hints = [hint.lower() for hint in CONTROL_TOPIC_HINTS]
    for topic in topics:
        topic_lower = topic.lower()
        if topic in seen:
            continue
        if (
            "bodyhub" in topic_lower
            and (
                "joint" in topic_lower
                or "position" in topic_lower
                or "servo" in topic_lower
                or topic_lower in lower_hints
            )
        ):
            candidates.append(topic)
            seen.add(topic)

    return candidates


def main():
    parser = argparse.ArgumentParser(
        description="Probe Roban BodyHub topics and JointControlPoint interfaces."
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT,
        help="Path for the probe log. Relative paths are resolved from the current directory.",
    )
    args = parser.parse_args()

    lines = []
    lines.append("Roban BodyHub interface probe")
    lines.append("output: %s" % os.path.abspath(args.output))

    commands = [
        "rostopic list | grep -i BodyHub",
        "rostopic list | grep -i Joint",
        "rostopic list | grep -i Position",
        "rostopic list | grep -i Servo",
        "rosmsg show bodyhub/JointControlPoint",
    ]

    for command in commands:
        append_section(lines, command, run_command(command))

    topics = get_topic_list()
    candidates = collect_candidate_topics(topics)

    append_section(lines, "candidate control topics", "\n".join(candidates))

    found_joint_controlpoint = []
    for topic in candidates:
        info = run_command("rostopic info %s" % topic)
        append_section(lines, "rostopic info %s" % topic, info)
        if "bodyhub/JointControlPoint" in info:
            found_joint_controlpoint.append(topic)
            lines.append("Found JointControlPoint topic: %s" % topic)

    if not found_joint_controlpoint:
        lines.append("")
        lines.append("No bodyhub/JointControlPoint topic found in candidate topics.")

    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    text = "\n".join(lines) + "\n"
    with open(args.output, "w") as f:
        f.write(text)

    print(text)


if __name__ == "__main__":
    main()
