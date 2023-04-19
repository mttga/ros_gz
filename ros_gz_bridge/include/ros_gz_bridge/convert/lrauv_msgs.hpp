#ifndef ROS_GZ_BRIDGE__CONVERT__LRAUV_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__LRAUV_MSGS_HPP_

// LRAUV-Gazebo Msgs
#include "lrauv_gazebo_plugins/lrauv_init.pb.h"
#include "lrauv_gazebo_plugins/lrauv_state.pb.h"
#include "lrauv_gazebo_plugins/lrauv_command.pb.h"
#include "lrauv_gazebo_plugins/lrauv_range_bearing_request.pb.h"
#include "lrauv_gazebo_plugins/lrauv_range_bearing_response.pb.h"

// ROS 2 messages
#include <lrauv_msgs/msg/lrauv_init.hpp>
#include <lrauv_msgs/msg/lrauv_state.hpp>
#include <lrauv_msgs/msg/lrauv_command.hpp>
#include <lrauv_msgs/msg/lrauv_range_bearing_request.hpp>
#include <lrauv_msgs/msg/lrauv_range_bearing_response.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{

// LRAUVInit
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVInit & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVInit & gz_msg);

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVInit & gz_msg,
  lrauv_msgs::msg::LRAUVInit & ros_msg);


// LRAUVState
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVState & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVState & gz_msg);

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVState & gz_msg,
  lrauv_msgs::msg::LRAUVState & ros_msg);


// LRAUVCommand
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVCommand & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVCommand & gz_msg);

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVCommand & gz_msg,
  lrauv_msgs::msg::LRAUVCommand & ros_msg);


// LRAUVRangeBearingRequest
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVRangeBearingRequest & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVRangeBearingRequest & gz_msg);

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVRangeBearingRequest & gz_msg,
  lrauv_msgs::msg::LRAUVRangeBearingRequest & ros_msg);


// LRAUVRangeBearingResponse
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVRangeBearingResponse & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVRangeBearingResponse & gz_msg);

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVRangeBearingResponse & gz_msg,
  lrauv_msgs::msg::LRAUVRangeBearingResponse & ros_msg);

}


#endif  // ROS_GZ_BRIDGE__CONVERT__LRAUV_MSGS_HPP_
