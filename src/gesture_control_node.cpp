#include <ros/ros.h>
// #include <quad_control/gesture_control_node.h>
#include "../include/gesture_control_node.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <cmath>
#include <string>

double GestureControl::GetYaw(geometry_msgs::Quaternion q) {
  double roll, pitch, yaw;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);
  return yaw;
}

void GestureControl::Initialize() {
  // wait for FCU connection
  ROS_INFO("Waiting for connecting to FCU.");
  while(ros::ok() && !_current_state.connected){
    ros::spinOnce();
    _rate.sleep();
  }
  ROS_INFO("Successfully connect to FCU");

  _offb_set_mode.request.custom_mode = "OFFBOARD";

  _arm_cmd.request.value = true;
  ROS_INFO("Armed.");

  _pose.position.x = 0;
  _pose.position.y = 0;
  _pose.position.z = 2;

  _lin_vel = 0.5;

  ROS_INFO("About to send setpoints.");
  for(int i = 100; ros::ok() && i > 0; --i){
    _local_pos_pub.publish(_pose);
    ros::spinOnce();
    _rate.sleep();
  }
  ROS_INFO("Finish sending setpoints. About to take off.");
}


void GestureControl::StateCB(const mavros_msgs::State::ConstPtr& msg) {
  _current_state = *msg;
}


void GestureControl::CurrPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  _current_pose = *msg;
}


void GestureControl::GestureCB(const gesture_recognition_client::Gesture::ConstPtr& msg) {
  if (msg->gesture_name == "left_up") {  // Go Up
    _pose.position.z = _current_pose.pose.position.z + 2;
  } else if (msg->gesture_name == "go_left" || msg->gesture_name == "go_right") {  // Turn left or turn right
    double yaw = GetYaw(_current_pose.pose.orientation);
    if (msg->gesture_name == "go_right") {
      _pose.yaw = yaw + 1.0;
    } else {
      _pose.yaw = yaw - 1.0;
    }
  } else if (msg->gesture_name == "left_up_right_up") {  // Move forward until calling stop
    double yaw = GetYaw(_current_pose.pose.orientation);
    _pose.position.x = _current_pose.pose.position.x + cos(yaw) * 10;
    _pose.position.y = _current_pose.pose.position.y + sin(yaw) * 10;
  } else if (msg->gesture_name == "equal") {  // Hover in the air
    _pose.position.x = _current_pose.pose.position.x;
    _pose.position.y = _current_pose.pose.position.y;
    _pose.position.z = _current_pose.pose.position.z;
  } else if (msg->gesture_name == "cross") {  // Land to the origin
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 0;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if (_land_client.call(srv_land)) {
      ROS_INFO("srv_land send ok %d", srv_land.response.success);
    } else {
      ROS_ERROR("Failed Land");
    }
  } else {
    ROS_INFO("Cannot define this action for now.");
  }
}


void GestureControl::Run() {
  while(ros::ok()){
    if(_current_state.mode != "OFFBOARD" &&
       (ros::Time::now() - _last_request > ros::Duration(5.0))) {
      if (_set_mode_client.call(_offb_set_mode) &&
         _offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      _last_request = ros::Time::now();
    } else {
      if (!_current_state.armed &&
         (ros::Time::now() - _last_request > ros::Duration(5.0))) {
        if (_arming_client.call(_arm_cmd) &&
           _arm_cmd.response.success){
          ROS_INFO("Vehicle armed");
        }
        _last_request = ros::Time::now();
      }
    }

    _local_pos_pub.publish(_pose);
    std::string mode_str = _current_state.mode;
    // ROS_INFO(mode_str.c_str());

    ros::spinOnce();
    _rate.sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gesture_control_node");
  GestureControl gc;
  gc.Initialize();
  gc.Run();

  return 0;
}
