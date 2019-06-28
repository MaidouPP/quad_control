#!/usr/bin/env python

import rospy
import math

from pyquaternion import Quaternion
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, Float64, String


class Px4Controller(object):
    def __init__(self):
        self.kDeadHeight = 1.0
        self.kDeadRange = 1.5
        rospy.init_node("px4_control_node")
        self.rate = rospy.Rate(10)

        self._local_pose = None
        self._state = None
        self._curr_heading = 0.0
        self._takeoff_height = 0.5
        self._armed = False
        self._landed = True
        self._offboard_mode = False

        # self.local_enu_position = None
        self._curr_target_pose = None
        self._frame = "BODY"
        self._state = None
        self._state_msg = None

        # Subscribers
        self._local_pose_sub = rospy.Subscriber("/mavros/local_position/pose",
                                                PoseStamped, self.LocalPoseCallback)
        self._state_sub = rospy.Subscriber("/mavros/state", State, self.MavrosStateCallback)
        self._target_position_sub = rospy.Subscriber("/offboard/set_pose/position",
                                                     PoseStamped, self.SetTargetPositionCallback)
        self._target_heading_sub = rospy.Subscriber("/offboard/set_pose/orientation",
                                                    Float32, self.SetTargetYawCallback)
        self._command_sub = rospy.Subscriber("/offboard/set_activity/type", String,
                                             self.CustomActivityCallback)

        # Publishers
        self._local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local',
                                                 PositionTarget, queue_size=10)

        # Service
        self._arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        print "Px4 Controller Initialized!"


    def Start(self, connection_wait_cycles=20, attempts_take_off=5,
              arm_wait_cycles=10, target_pose_cycles=60):
        for i in range(connection_wait_cycles):
            if self._state_msg and self._state_msg.connected:
                break
            rospy.loginfo("Waiting for px4 connection .. %d", i)
            self.rate.sleep()
        self._curr_target_pose = self._ConstructTarget(0, 0, self._takeoff_height, self._curr_heading)

        # Send way points and switch to arm+offboard to prepare for flying
        for i in range(attempts_take_off):
            rospy.loginfo("Trying : %d", i)

            # Arm the motors
            self._armed = self.EnableArm()
            for j in range(arm_wait_cycles):
                if self._state_msg and self._state_msg.armed:
                    break
                rospy.loginfo("Waiting for arming ... %d", j)
                self.rate.sleep()

            # Enable offboard mode
            self._offboard_mode = self.EnableOffboard()
            if not self._offboard_mode:
                continue
            self.rate.sleep()

            # Take off command
            self._local_target_pub.publish(self._curr_target_pose)
            for _ in range(target_pose_cycles):
                self._local_target_pub.publish(self._curr_target_pose)
                if self._local_pose and abs(self._local_pose.pose.position.z -
                                            self._takeoff_height) < 0.1:
                    break
                rospy.loginfo("Taking off ... z=%f",
                              (self._local_pose.pose.position.z if
                               self._local_pose else -100))
                self.rate.sleep()

            # Exit the main loop if done
            if self._local_pose and abs(self._local_pose.pose.position.z -
                                        self._takeoff_height) < 0.1:
                break

        if self._local_pose is None:
            rospy.logerr("Local pose is None. Disarm now!")
            self.EnableDisarm()

        if self.CheckIfTakeoffed():
            rospy.loginfo("Vehicle Took Off!")
        else:
            rospy.logerr("Vehicle Failed taking off!")
            #return

        # Control loop
        while self._armed and self._offboard_mode and not rospy.is_shutdown():
            self._local_target_pub.publish(self._curr_target_pose)
            if self._local_pose.pose.position.z < 0.15:
                if not self._mode_service(custom_mode="AUTO.LAND"):
                    rospy.logerr("Landing mode set failed")
                if self.EnableDisarm():
                    self.state = "DISARMED"
            self.rate.sleep()


    def _ConstructTarget(self, x, y, z, yaw, yaw_rate=1):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        """
        reference: https://github.com/PX4/Devguide/blob/master/en/ros/external_position_estimation.md

        PX4 uses FRD (X Forward, Y Right and Z Down) for the local body frame,
        and NED (X North, Y East, Z Down) for the local world frame -
        set in MAVLink using MAV_FRAME_BODY_OFFSET_NED and MAV_FRAME_LOCAL_NED, respectively.

        uint8 coordinate_frame
        FRAME_LOCAL_NED = 1
        FRAME_LOCAL_OFFSET_NED = 7
        FRAME_BODY_NED = 8
        FRAME_BODY_OFFSET_NED = 9
        """
        target_raw_pose.coordinate_frame = 9

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = 0 # yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose


    def LocalPoseCallback(self, msg):
        self._local_pose = msg
        ori = msg.pose.orientation
        q = Quaternion(ori.w, ori.x, ori.y, ori.z)
        self._curr_heading = q.yaw_pitch_roll[0]

        # For safety
        if (msg.pose.position.z > self.kDeadHeight or \
            math.fabs(msg.pose.position.x) > self.kDeadRange or \
            math.fabs(msg.pose.position.y) > self.kDeadRange
           ):
            rospy.loginfo("Out of safety range. Landing... %f, %f, %f", msg.pose.position.x,
                          msg.pose.position.y, msg.pose.position.z)
            mode_sent = self._mode_service(custom_mode="AUTO.LAND")
            rospy.loginfo("AUTO.LAND: " + str(mode_sent))
            self._curr_target_pose = self._ConstructTarget(self._local_pose.pose.position.x,
                                                           self._local_pose.pose.position.y,
                                                           0.1,
                                                           self._curr_heading)


    def MavrosStateCallback(self, msg):
        self._state_msg = msg
        self._state = msg.mode


    def Body2Enu(self, body_target_x, body_target_y, body_target_z):
        ENU_x = body_target_y
        ENU_y = -body_target_x
        ENU_z = body_target_z
        return ENU_x, ENU_y, ENU_z


    def BodyEnu2OffsetEnu(self, msg):
        FLU_x = msg.pose.position.x * math.cos(self._curr_heading) - msg.pose.position.y * math.sin(self._curr_heading)
        FLU_y = msg.pose.position.x * math.sin(self._curr_heading) + msg.pose.position.y * math.cos(self._curr_heading)
        FLU_z = msg.pose.position.z
        return FLU_x, FLU_y, FLU_z


    def SetTargetPositionCallback(self, msg):
        """
        See reference https://dev.px4.io/en/ros/external_position_estimation.html
        """
        if msg.header.frame_id == 'base_link':
            # BODY_OFFSET_ENU
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body
            self._frame = "BODY"
            x, y, z = self.BodyEnu2OffsetEnu(msg)

            body_x = x + self._local_pose.pose.position.x
            body_y = y + self._local_pose.pose.position.y
            body_z = z + self._local_pose.pose.position.z

            self._curr_target_pose = self._ConstructTarget(body_x,
                                                           body_y,
                                                           body_z,
                                                           self._curr_heading)
        else:
            # LOCAL_ENU
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X
            self._frame = "LOCAL_ENU"
            # ENU_x, ENU_y, ENU_z = self.Body2Enu(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            # enu_x, enu_y, enu_z = msg.pose.position

            self._curr_target_pose = self._ConstructTarget(msg.pose.position.x,
                                                           msg.pose.position.y,
                                                           msg.pose.position.z,
                                                           self._curr_heading)
        pos = self._curr_target_pose.position
        rospy.loginfo( "Received New Position Task! Set current target pose to: %f, %f, %f",
                      pos.x, pos.y, pos.z)


    def CustomActivityCallback(self, msg):
        print "Received Custom Activity:", msg.data

        if msg.data == "LAND":
            print "LANDING!"
            self._mode_service(custom_mode = "AUTO.LAND")
            self._curr_target_pose = self._ConstructTarget(self._local_pose.pose.position.x,
                                                           self._local_pose.pose.position.y,
                                                           0.0,
                                                           self._curr_heading)
        elif msg.data == "HOVER":
            print "HOVERING!"
            self._state = "HOVER"
            self.EnableHover()
        else:
            print "Received Custom Activity:", msg.data, "not supported yet!"


    def SetTargetYawCallback(self, msg):
        yaw_deg = msg.data * math.pi / 180.0
        self._curr_target_pose = self._ConstructTarget(self._local_pose.pose.position.x,
                                                       self._local_pose.pose.position.y,
                                                       self._local_pose.pose.position.z,
                                                       yaw_deg)
        print "Received orientation command! Set current target pose to: ", self._curr_target_pose


    def EnableArm(self):
        if self._arm_service(True):
            return True
        else:
            print "Vehicle arming failed!"
            return False


    def EnableDisarm(self):
        if self._arm_service(False):
            return True
        else:
            print "Vehicle disarming failed!"
            return False


    def EnableOffboard(self):
        if self._mode_service(custom_mode='OFFBOARD'):
            return True
        else:
            print "Vechile Offboard failed"
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
            print self._offboard_mode, " ", self._armed
            return False


if __name__ == '__main__':

    con = Px4Controller()
    con.Start()
