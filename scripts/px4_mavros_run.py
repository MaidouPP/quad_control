#!/usr/bin/env python

import rospy
import math

from pyquaternion import Quaternion
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, Float64, String


class Px4Controller:

    def __init__(self):

        self._local_pose = None
        self._state = None
        self._curr_heading = 0.0
        self._takeoff_height = 1.0
        self._armed = False
        self._landed = True
        self._offboard_mode = False

        # self.local_enu_position = None
        self._curr_target_pose = None
        self._frame = "BODY"
        self._state = None

        # Subscribers
        self._local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.LocalPoseCallback)
        self._state_sub = rospy.Subscriber("/mavros/state", State, self.MavrosStateCallback)
        self._target_position_sub = rospy.Subscriber("/offboard/set_pose/position", PoseStamped, self.SetTargetPositionCallback)
        self._target_heading_sub = rospy.Subscriber("/offboard/set_pose/orientation", Float32, self.SetTargetYawCallback)
        self._command_sub = rospy.Subscriber("/offboard/set_activity/type", String, self.CustomActivityCallback)

        # Publishers
        self._local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Service
        self._arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")


    def Start(self):
        rospy.init_node("px4_control_node")
        rate = rospy.Rate(20)

        self._curr_target_pose = self._ConstructTarget(0, 0, self._takeoff_height, self._curr_heading)

        # print("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        # Send way points and switch to arm+offboard to prepare for flying
        for i in range(10):
            self._local_target_pub.publish(self._curr_target_pose)
            self._armed = self.EnableArm()
            self._offboard_mode = self.EnableOffboard()
            rate.sleep()

        if self.CheckIfTakeoffed():
            print("Vehicle Took Off!")
        else:
            print("Vehicle Failed taking off!")
            return

        # Control loop
        while self._armed and self._offboard_mode and not rospy.is_shutdown():
            self._local_target_pub.publish(self._curr_target_pose)
            if self._state == "LAND" and self._local_pose.pose.position.z < 0.15:
                if not self.EnableDisarm():
                    self.state = "DISARMED"
            rate.sleep()


    def _ConstructTarget(self, x, y, z, yaw, yaw_rate=1):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        # uint8 coordinate_frame
        # FRAME_LOCAL_NED = 1
        # FRAME_LOCAL_OFFSET_NED = 7
        # FRAME_BODY_NED = 8
        # FRAME_BODY_OFFSET_NED = 9
        target_raw_pose.coordinate_frame = 9

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose


    def LocalPoseCallback(self, msg):
        self._local_pose = msg
        ori = msg.pose.orientation
        q = Quaternion(ori.w, ori.x, ori.y, ori.z)
        self._curr_heading = q.yaw_pitch_roll[0]


    # def GetCurrHeading(self):
    #     ori = self._local_pose.pose.orientation
    #     q = Quaternion(ori.w, ori.x, ori.y, ori.z)
    #     self._curr_heading = q.yaw_pitch_roll[0]


    def MavrosStateCallback(self, msg):
        self._state = msg.mode


    def Body2Enu(self, body_target_x, body_target_y, body_target_z):
        ENU_x = body_target_y
        ENU_y = -body_target_x
        ENU_z = body_target_z

        return ENU_x, ENU_y, ENU_z


    def BodyEnu2Flu(self, msg):

        FLU_x = msg.pose.position.x * math.cos(self._curr_heading) - msg.pose.position.y * math.sin(self._curr_heading)
        FLU_y = msg.pose.position.x * math.sin(self._curr_heading) + msg.pose.position.y * math.cos(self._curr_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z


    def SetTargetPositionCallback(self, msg):
        print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_OFFSET_ENU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self._frame = "BODY"
            print("body FLU frame")
            FLU_x, FLU_y, FLU_z = self.BodyEnu2Flu(msg)

            body_x = FLU_x + self._local_pose.pose.position.x
            body_y = FLU_y + self._local_pose.pose.position.y
            body_z = FLU_z + self._local_pose.pose.position.z

            self._curr_target_pose = self._ConstructTarget(body_x,
                                                            body_y,
                                                            body_z,
                                                            self._curr_heading)
        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X
            self._frame = "LOCAL_ENU"
            print("local ENU frame")
            ENU_x, ENU_y, ENU_z = self.Body2Enu(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

            self._curr_target_pose = self._ConstructTarget(ENU_x,
                                                           ENU_y,
                                                           ENU_z,
                                                           self._curr_heading)

    '''
    Receive A Custom Activity
    '''
    def CustomActivityCallback(self, msg):
        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self._state = "LAND"
            self._curr_target_pose = self._ConstructTarget(self._local_pose.pose.position.x,
                                                           self._local_pose.pose.position.y,
                                                           0.1,
                                                           self._curr_heading)
        elif msg.data == "HOVER":
            print("HOVERING!")
            self._state = "HOVER"
            self.EnableHover()
        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    def SetTargetYawCallback(self, msg):
        print("Received orientatino command!")

        yaw_deg = msg.data * math.pi / 180.0
        self._curr_target_pose = self._ConstructTarget(self._local_pose.pose.position.x,
                                                       self._local_pose.pose.position.y,
                                                       self._local_pose.pose.position.z,
                                                       yaw_deg)


    def EnableArm(self):
        if self._arm_service(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False


    def EnableDisarm(self):
        if self._arm_service(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False


    def EnableOffboard(self):
        if self._mode_service(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False


    def EnableHover(self):
        self._curr_target_pose = self._ConstructTarget(self._local_pose.pose.position.x,
                                                       self._local_pose.pose.position.y,
                                                       self._local_pose.pose.position.z,
                                                       self._curr_heading)


    def CheckIfTakeoffed(self):
        if self._local_pose.pose.position.z > 0.1 and self._offboard_mode and self._armed:
            return True
        else:
            print(self._offboard_mode, " ", self._armed)
            return False


if __name__ == '__main__':

    con = Px4Controller()
    con.Start()
