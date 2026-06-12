# -*- coding: utf-8 -*-

import argparse
import json
import os
import shlex
import subprocess
import time

import cv2

from pose_detector import (
    UpperBodyPoseDetector,
    calculate_arm_raise_angle,
    draw_pose_result_overlay,
)
from smoothing_filter import AngleSmoother


DEFAULT_ROBOT_IP = "10.129.44.37"
DEFAULT_REMOTE_USER = "lemon"
DEFAULT_STREAM_URL = (
    "http://10.129.44.37:8080/stream?topic=/camera/color/image_raw"
)

LOCAL_JSON_TMP_PATH = "logs/week4/upper_body_pose_angles.tmp.json"
LOCAL_JSON_PATH = "logs/week4/upper_body_pose_angles.json"
LOCAL_LOG_PATH = "logs/week4/mac_stream_pose_json_sender_log.txt"
LOCAL_OVERLAY_PATH = "images/week4/stream_pose_overlay.jpg"

DEFAULT_REMOTE_JSON_PATH = (
    "/home/lemon/roban_motion_control/week4/logs/upper_body_pose_angles.json"
)

SOURCE_NAME = "roban_video_stream"


def ensure_dirs():
    os.makedirs(os.path.dirname(LOCAL_JSON_TMP_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(LOCAL_JSON_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(LOCAL_LOG_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(LOCAL_OVERLAY_PATH), exist_ok=True)


def run_cmd(cmd):
    completed = subprocess.run(
        cmd,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    if completed.returncode != 0:
        print("Command failed:", cmd)
        print("stderr:", completed.stderr.strip())

    return completed.returncode == 0


def atomic_write_local_json(data):
    with open(LOCAL_JSON_TMP_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    os.replace(LOCAL_JSON_TMP_PATH, LOCAL_JSON_PATH)


def push_json_to_robot(local_json_path, remote_user, robot_ip, remote_json_path):
    remote_tmp_path = remote_json_path.replace(".json", ".tmp.json")
    remote_host = "%s@%s" % (remote_user, robot_ip)

    scp_cmd = "scp %s %s:%s" % (
        local_json_path,
        remote_host,
        remote_tmp_path
    )

    mv_cmd = "ssh %s 'mv %s %s'" % (
        remote_host,
        remote_tmp_path,
        remote_json_path
    )

    if not run_cmd(scp_cmd):
        return False

    if not run_cmd(mv_cmd):
        return False

    return True


class RemoteJsonStreamWriter(object):
    def __init__(self, remote_user, robot_ip, remote_json_path):
        self.remote_user = remote_user
        self.robot_ip = robot_ip
        self.remote_json_path = remote_json_path
        self.proc = None

    def start(self):
        remote_host = "%s@%s" % (self.remote_user, self.robot_ip)
        remote_tmp_path = self.remote_json_path.replace(".json", ".tmp.json")
        script = """
import os
import sys

path = %r
tmp = %r
directory = os.path.dirname(path)
if directory and not os.path.exists(directory):
    try:
        os.makedirs(directory)
    except OSError:
        pass

for line in sys.stdin:
    line = line.rstrip("\\n")
    if not line:
        continue
    f = open(tmp, "w")
    f.write(line + "\\n")
    f.close()
    os.rename(tmp, path)
""" % (self.remote_json_path, remote_tmp_path)

        remote_cmd = "python -u -c %s" % shlex.quote(script)
        self.proc = subprocess.Popen(
            ["ssh", remote_host, remote_cmd],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )

    def close(self):
        if self.proc is None:
            return
        try:
            if self.proc.stdin:
                self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.terminate()
        except Exception:
            pass
        self.proc = None

    def push(self, data):
        if self.proc is None or self.proc.poll() is not None:
            self.close()
            self.start()

        line = json.dumps(
            data,
            ensure_ascii=False,
            separators=(",", ":"),
        )

        try:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
            return True
        except Exception:
            self.close()
            return False


def apply_raise_angle_source(result, source):
    if source == "upper-arm" or not result.get("visible", False):
        result["raise_angle_source"] = source
        return result

    updated = dict(result)
    updated["left_upper_arm_raise_angle"] = result.get("left_arm_raise_angle")
    updated["right_upper_arm_raise_angle"] = result.get("right_arm_raise_angle")

    left_hand_raise = calculate_arm_raise_angle(
        result["left_shoulder"],
        result["left_wrist"],
    )
    right_hand_raise = calculate_arm_raise_angle(
        result["right_shoulder"],
        result["right_wrist"],
    )

    updated["left_hand_raise_angle"] = left_hand_raise
    updated["right_hand_raise_angle"] = right_hand_raise

    if source == "wrist":
        if left_hand_raise is not None:
            updated["left_arm_raise_angle"] = left_hand_raise
        if right_hand_raise is not None:
            updated["right_arm_raise_angle"] = right_hand_raise

    updated["raise_angle_source"] = source
    return updated


def build_publish_json(smoothed_result):
    timestamp = time.time()

    if not smoothed_result.get("visible", False):
        return {
            "visible": False,
            "confidence": float(smoothed_result.get("confidence", 0.0)),
            "left_arm_raise_angle": None,
            "left_elbow_angle": None,
            "right_arm_raise_angle": None,
            "right_elbow_angle": None,
            "reason": smoothed_result.get(
                "reason",
                "low visibility or no person"
            ),
            "raise_angle_source": smoothed_result.get(
                "raise_angle_source",
                "unknown",
            ),
            "source": SOURCE_NAME,
            "timestamp": timestamp
        }

    data = {
        "visible": True,
        "confidence": float(smoothed_result.get("confidence", 0.0)),
        "left_arm_raise_angle": smoothed_result.get(
            "smoothed_left_arm_raise_angle",
            smoothed_result.get("left_arm_raise_angle")
        ),
        "left_elbow_angle": smoothed_result.get(
            "smoothed_left_elbow_angle",
            smoothed_result.get("left_elbow_angle")
        ),
        "right_arm_raise_angle": smoothed_result.get(
            "smoothed_right_arm_raise_angle",
            smoothed_result.get("right_arm_raise_angle")
        ),
        "right_elbow_angle": smoothed_result.get(
            "smoothed_right_elbow_angle",
            smoothed_result.get("right_elbow_angle")
        ),
        "reason": smoothed_result.get("reason", "ok"),
        "raise_angle_source": smoothed_result.get(
            "raise_angle_source",
            "unknown",
        ),
        "source": SOURCE_NAME,
        "timestamp": timestamp
    }

    for field in [
        "left_upper_arm_raise_angle",
        "right_upper_arm_raise_angle",
        "left_hand_raise_angle",
        "right_hand_raise_angle",
        "left_arm_raise_angle",
        "right_arm_raise_angle",
    ]:
        if field in smoothed_result:
            data["raw_" + field] = smoothed_result.get(field)

    return data


def format_angle(value):
    if value is None:
        return "None"
    return "%.2f" % value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_ROBOT_IP,
        help="Roban robot IP"
    )
    parser.add_argument(
        "--remote-user",
        default=DEFAULT_REMOTE_USER,
        help="Roban SSH user"
    )
    parser.add_argument(
        "--stream-url",
        default=DEFAULT_STREAM_URL,
        help="Roban web_video_server stream URL"
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="pose recognition and JSON upload frequency"
    )
    parser.add_argument(
        "--remote-json",
        default=DEFAULT_REMOTE_JSON_PATH,
        help="remote JSON path on Roban"
    )
    parser.add_argument(
        "--min-visibility",
        type=float,
        default=0.6,
        help="minimum keypoint visibility threshold"
    )
    parser.add_argument(
        "--overlay-interval",
        type=int,
        default=10,
        help="save overlay image every N processed frames"
    )
    parser.add_argument(
        "--upload-mode",
        choices=["ssh-stream", "scp"],
        default="ssh-stream",
        help="Remote JSON upload mode. ssh-stream keeps one SSH process open.",
    )
    parser.add_argument(
        "--smoothing-alpha",
        type=float,
        default=0.45,
        help="Angle smoothing alpha. Higher is faster, lower is smoother.",
    )
    parser.add_argument(
        "--raise-angle-source",
        choices=["wrist", "upper-arm"],
        default="wrist",
        help="Use shoulder-to-wrist or shoulder-to-elbow for arm raise angle.",
    )
    args = parser.parse_args()
    args.smoothing_alpha = max(0.0, min(1.0, args.smoothing_alpha))

    ensure_dirs()

    print("Mac stream pose JSON sender started.")
    print("Stream URL:", args.stream_url)
    print("Robot:", "%s@%s" % (args.remote_user, args.robot_ip))
    print("Remote JSON:", args.remote_json)
    print("Target frequency: %.1f Hz" % args.hz)
    print("Min visibility:", args.min_visibility)
    print("Upload mode:", args.upload_mode)
    print("Smoothing alpha:", args.smoothing_alpha)
    print("Raise angle source:", args.raise_angle_source)

    cap = cv2.VideoCapture(args.stream_url)
    if not cap.isOpened():
        print("Failed to open Roban video stream.")
        print("Possible reasons:")
        print("1. web_video_server is not running on Roban.")
        print("2. Mac and Roban are not in the same network.")
        print("3. Robot IP or stream URL is wrong.")
        print("4. Try URL with &type=mjpeg.")
        return

    smoother = AngleSmoother(alpha=args.smoothing_alpha)
    detector = UpperBodyPoseDetector(min_detection_confidence=0.6, min_tracking_confidence=0.7)
    frame_count = 0
    loop_interval = 1.0 / max(args.hz, 0.1)
    writer = None
    if args.upload_mode == "ssh-stream":
        writer = RemoteJsonStreamWriter(
            args.remote_user,
            args.robot_ip,
            args.remote_json,
        )

    try:
        with open(LOCAL_LOG_PATH, "a", encoding="utf-8") as log_file:
            while True:
                start_time = time.time()

                ret, frame = cap.read()
                if not ret or frame is None:
                    print("Failed to read frame from stream.")
                    time.sleep(loop_interval)
                    continue

                raw_result = detector.detect(
                    frame,
                    min_visibility=args.min_visibility
                )
                raw_result = apply_raise_angle_source(
                    raw_result,
                    args.raise_angle_source,
                )

                smoothed_result = smoother.update_result(raw_result)
                publish_json = build_publish_json(smoothed_result)

                atomic_write_local_json(publish_json)

                if writer is not None:
                    push_ok = writer.push(publish_json)
                    if not push_ok:
                        push_ok = push_json_to_robot(
                            LOCAL_JSON_PATH,
                            args.remote_user,
                            args.robot_ip,
                            args.remote_json
                        )
                else:
                    push_ok = push_json_to_robot(
                        LOCAL_JSON_PATH,
                        args.remote_user,
                        args.robot_ip,
                        args.remote_json
                    )

                if frame_count % max(args.overlay_interval, 1) == 0:
                    overlay = draw_pose_result_overlay(frame, raw_result)
                    cv2.imwrite(LOCAL_OVERLAY_PATH, overlay)

                now = time.time()
                elapsed = now - start_time
                actual_hz = 1.0 / elapsed if elapsed > 1e-6 else 0.0

                line = (
                    "visible={visible} conf={conf:.3f} "
                    "L_raise={lra} L_elbow={le} "
                    "R_raise={rra} R_elbow={re} "
                    "push_ok={push_ok} actual_hz={hz:.2f} "
                    "reason={reason}"
                ).format(
                    visible=publish_json["visible"],
                    conf=publish_json["confidence"],
                    lra=format_angle(publish_json["left_arm_raise_angle"]),
                    le=format_angle(publish_json["left_elbow_angle"]),
                    rra=format_angle(publish_json["right_arm_raise_angle"]),
                    re=format_angle(publish_json["right_elbow_angle"]),
                    push_ok=push_ok,
                    hz=actual_hz,
                    reason=publish_json["reason"]
                )

                print(line)
                log_file.write(json.dumps(publish_json, ensure_ascii=False) + "\n")
                log_file.flush()

                frame_count += 1

                sleep_time = max(0.0, loop_interval - elapsed)
                time.sleep(sleep_time)
    finally:
        detector.close()
        cap.release()
        if writer is not None:
            writer.close()


if __name__ == "__main__":
    main()
