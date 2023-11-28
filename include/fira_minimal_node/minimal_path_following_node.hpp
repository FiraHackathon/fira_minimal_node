// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_HPP_

// std
#include <atomic>
#include <functional>
#include <memory>
#include <queue>

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joy.hpp"

//romea
#include "romea_core_common/concurrency/SharedVariable.hpp"
#include "romea_mobile_base_utils/control/command_interface.hpp"
#include "romea_mobile_base_utils/control/command_traits.hpp"
#include "romea_path_msgs/msg/path_matching_info2_d.hpp"
#include "romea_path_utils/path_matching_info_conversions.hpp"
#include "romea_core_path_following/PathFollowing.hpp"

namespace romea
{
namespace ros2
{


template<class CommandType>
class MinimalPathFollowingNode
{
public:
  using Node = rclcpp::Node;
  using VehicleInterface = CommandInterface<CommandType>;
  using OdometryMeasure = typename CommandTraits<CommandType>::Measure;
  using OdometryMeasureMsg = typename CommandTraits<CommandType>::MeasureMsg;

public:
  explicit MinimalPathFollowingNode(const rclcpp::NodeOptions & options);

  virtual ~MinimalPathFollowingNode() = default;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  void process_matching_info_(romea_path_msgs::msg::PathMatchingInfo2D::ConstSharedPtr msg);

  void process_odometry_(const OdometryMeasureMsg & msg);

  void process_joystick_(sensor_msgs::msg::Joy::ConstSharedPtr msg);

protected:
  Node::SharedPtr node_;

  core::PathFollowingSetPoint setpoint_;
  core::SharedVariable<OdometryMeasure> odometry_measure_;
  std::unique_ptr<core::PathFollowingBase<CommandType>> path_following_;

  std::unique_ptr<VehicleInterface> cmd_interface_;
  rclcpp::SubscriptionBase::SharedPtr matching_sub_;
  rclcpp::SubscriptionBase::SharedPtr odometry_sub_;
  rclcpp::SubscriptionBase::SharedPtr joystick_sub_;

  int joy_start_button_id_;
  int joy_stop_button_id_;
};

using TwoAxleSteeringMinimalPathFollowingNode =
  MinimalPathFollowingNode<core::TwoAxleSteeringCommand>;
using OneAxleSteeringMinimalPathFollowingNode =
  MinimalPathFollowingNode<core::OneAxleSteeringCommand>;

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_HPP_
