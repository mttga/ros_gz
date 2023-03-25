#ifndef ROS_GZ_BRIDGE__CONVERT__LRAUV_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__LRAUV_MSGS_HPP_

// LRAUV-Gazebo Msgs
#include "lrauv_gazebo_plugins/lrauv_init.pb.h"
#include "lrauv_gazebo_plugins/lrauv_state.pb.h"

// ROS 2 messages
#include <lrauv_msgs/msg/lrauv_init.hpp>
#include <lrauv_msgs/msg/lrauv_state.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{
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
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__LRAUV_MSGS_HPP_
