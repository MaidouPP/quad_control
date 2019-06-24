#!/usr/bin/env python

import rospy
import sys

from commander import Commander
from gesture_recognition_client.msg import Gesture


class GestureCommander(Commander):
    def __init__(self):
        super(GestureCommander, self).__init__()
        self._ges_sub = rospy.Subscriber("/gesture_recognition/gesture", Gesture, self.GestureCallback)


    def GestureCallback(self, msg):
        if msg.gesture_name == "left_up":
            self.Move(0, 0, 0.2)
        elif msg.gesture_name == "right_up":
            self.Move(0, 0, -0.2)
        elif msg.gesture_name == "go_left":
            self.Turn(30)
        elif msg.gesture_name == "go_right":
            self.Turn(-30)
        elif msg.gesture_name == "left_up_right_up":
            self.Move(0.2, 0, 0)
        elif msg.gesture_name == "cross":
            self.Land()
        else:
            self.Hover()


    def Run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


def main(argv):
    gcom = GestureCommander()
    gcom.Run()


if __name__ == "__main__":
    main(sys.argv)
