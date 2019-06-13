#!/usr/bin/env python

import numpy as np
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math


class Commander(object):
    def __init__(self):
        rospy.init_node("commander_node")
        self.rate = rospy.Rate(20)
        self.position_target_pub = rospy.Publisher('/offboard/set_pose/position', PoseStamped, queue_size=1)
        self.yaw_target_pub = rospy.Publisher('/offboard/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('/offboard/set_activity/type', String, queue_size=10)


    def Move(self, x, y, z, BODY_OFFSET_ENU=False):
        for _ in range(3):
            self.position_target_pub.publish(self.SetPose(x, y, z, BODY_OFFSET_ENU=BODY_OFFSET_ENU))


    def Turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)


    # land at current position
    def Land(self):
        self.custom_activity_pub.publish(String("LAND"))


    # hover at current position
    def Hover(self):
        self.custom_activity_pub.publish(String("HOVER"))


    # return to home position with defined height
    def ReturnHome(self, height):
        self.position_target_pub.publish(self.SetPose(0, 0, height, False))


    def SetPose(self, x, y, z, BODY_OFFSET_ENU=True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention

        if BODY_OFFSET_ENU:
            pose.header.frame_id = 'base_link'
        else:
            pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        return pose

eight_trajectory = np.array([
        [-0.3, -0.3, 0.3],
        [-0.1, -0.3, 0.3],
        [ 0.1,  0.3, 0.3],
        [ 0.3,  0.3, 0.3],
        [ 0.3, -0.3, 0.3],
        [ 0.1, -0.3, 0.3],
        [-0.1,  0.3, 0.3],
        [-0.3,  0.3, 0.3],
        [-0.3, -0.3, 0.3]])

five_nine_traj = np.array([
    [ 0.0,  0.0,  0.5],
    [-1.5,  0.0,  0.5],
    [-1.5, -2.7,  0.5]])


if __name__ == "__main__":
    con = Commander()

    for _ in range(5):
        for pos in eight_trajectory:
            con.Move(pos[0], pos[1], pos[2])
            time.sleep(2)
    time.sleep(2)
    con.Land()
    time.sleep(2)
    con.Move(0, 0, 0)
