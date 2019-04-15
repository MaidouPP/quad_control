#include <ros/ros.h>
// #include <quad_control/gesture_control_node.h>
#include "../include/gesture_control_node.h"
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

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

  _pose.pose.position.x = 0;
  _pose.pose.position.y = 0;
  _pose.pose.position.z = 2;

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


void GestureControl::GestureCB(const gesture_recognition_client::Gesture::ConstPtr& msg) {
  if (msg->gesture_name == "left_up") {
    _pose.pose.position.z += 2;
  } else if (msg->gesture_name == "go_left" || msg->gesture_name == "go_right") {
    tf2::Quaternion q_orig, q_rot, q_new;
    double r = 0, p = 0, y = 0;
    tf2::convert(_pose.pose.orientation, q_orig);
    if (msg->gesture_name == "go_right") {
      y = 1.0;  // yaw by around 30 degree
    } else {
      y = -1.0;
    }
    q_rot.setRPY(r, p, y);
    q_new = q_orig * q_rot;  // Calculate the new orientation
    q_new.normalize();
    tf2::convert(q_new, _pose.pose.orientation);
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
      if(!_current_state.armed &&
         (ros::Time::now() - _last_request > ros::Duration(5.0))){
        if(_arming_client.call(_arm_cmd) &&
           _arm_cmd.response.success){
          ROS_INFO("Vehicle armed");
        }
        _last_request = ros::Time::now();
      }
    }

    _local_pos_pub.publish(_pose);
    std::string mode_str = _current_state.mode;
    ROS_INFO(mode_str.c_str());

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
