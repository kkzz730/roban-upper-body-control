#!/usr/bin/env python
# -*- coding: utf-8 -*-

from lejulib import *
import rospy

def main():
    node_initial()

    try:
        print("Week3 static angle test: conservative right arm action frame")

        frame_1 = [0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0]
        frame_2 = [0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,40,28,0,0,0,0]
        frame_3 = frame_1[:]

        frames = [
            (frame_1, 800, 200),
            (frame_2, 800, 300),
            (frame_3, 800, 0)
        ]

        client_action.action(frames)
        rospy.signal_shutdown("done")
        rospy.spin()

    except Exception as err:
        serror(err)

if __name__ == "__main__":
    main()
