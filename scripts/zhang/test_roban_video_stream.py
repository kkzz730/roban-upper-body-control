# -*- coding: utf-8 -*-

import argparse
import os
import time
import cv2


DEFAULT_URL = "http://10.129.44.37:8080/stream?topic=/camera/color/image_raw"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--url",
        default=DEFAULT_URL,
        help="Roban web_video_server stream URL"
    )
    parser.add_argument(
        "--save",
        default="images/week4/test_roban_stream.jpg",
        help="Path to save one test frame"
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=100,
        help="Maximum frames to try reading"
    )
    args = parser.parse_args()

    print("Trying to open Roban video stream:")
    print(args.url)

    cap = cv2.VideoCapture(args.url)

    if not cap.isOpened():
        print("Failed to open video stream.")
        print("Possible reasons:")
        print("1. web_video_server is not running on Roban.")
        print("2. Mac and Roban are not in the same network.")
        print("3. The stream URL is wrong.")
        print("4. Try adding type=mjpeg:")
        print("   http://10.129.44.37:8080/stream?topic=/camera/color/image_raw&type=mjpeg")
        print("5. Try compressed topic:")
        print("   http://10.129.44.37:8080/stream?topic=/camera/color/image_raw/compressed")
        return

    os.makedirs(os.path.dirname(args.save), exist_ok=True)

    start_time = time.time()
    success = False

    for i in range(args.max_frames):
        ret, frame = cap.read()

        if not ret or frame is None:
            print("Frame %d: ret=False, trying next frame..." % i)
            continue

        elapsed = time.time() - start_time
        print("Frame %d: ret=True, shape=%s, elapsed=%.2fs" % (
            i,
            str(frame.shape),
            elapsed
        ))

        ok = cv2.imwrite(args.save, frame)

        if ok:
            print("Saved test frame to:", args.save)
        else:
            print("Failed to save frame to:", args.save)

        success = True
        break

    cap.release()

    if not success:
        print("Failed to read a valid frame within %d frames." % args.max_frames)
        print("Please check web_video_server, network, robot IP, and stream URL.")
    else:
        print("Video stream test passed.")


if __name__ == "__main__":
    main()
