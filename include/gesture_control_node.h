#ifndef GESTURE_CONTROL_NODE_H
#define GESTURE_CONTROL_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <gesture_recognition_client/Gesture.h>


class GestureControl{
 private:
  mavros_msgs::State _current_state;
  mavros_msgs::CommandBool _arm_cmd;
  geometry_msgs::PoseStamped _pose;
  ros::NodeHandle _nh;
  ros::Subscriber _state_sub = _nh.subscribe<mavros_msgs::State>
      ("mavros/state", 10, &GestureControl::StateCB, this);
  ros::Subscriber _gesture_sub = _nh.subscribe<gesture_recognition_client::Gesture>
      ("gesture_recognition/gesture", 10, &GestureControl::GestureCB, this);
  ros::Publisher _local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>
      ("mavros/setpoint_position/local", 10);
  ros::ServiceClient _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>
      ("mavros/cmd/arming");
  ros::ServiceClient _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>
      ("mavros/set_mode");
  ros::Rate _rate;
  ros::Time _last_request = ros::Time::now();
  mavros_msgs::SetMode _offb_set_mode;

 public:
  GestureControl() : _rate(20.0), _pose() {};
  void Initialize();
  // state_sub callback function
  void StateCB(const mavros_msgs::State::ConstPtr& msg);
  // gesture_sub callback function
  void GestureCB(const gesture_recognition_client::Gesture::ConstPtr& msg);
  void Run();
};


#endif
