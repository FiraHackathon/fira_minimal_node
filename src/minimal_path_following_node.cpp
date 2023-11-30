// std
// #include <functional>
// #include <type_traits>


#include "fira_minimal_node/minimal_path_following_node.hpp"
#include "fira_minimal_node/command_factory.hpp"
#include "romea_path_utils/path_matching_info_conversions.hpp"
#include "romea_common_utils/conversions/twist2d_conversions.hpp"
#include "romea_mobile_base_utils/conversions/kinematic_conversions.hpp"
#include "romea_mobile_base_utils/params/command_interface_parameters.hpp"


namespace
{
const int XBOX_X_BUTTON = 2;
const int XBOX_B_BUTTON = 1;
}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<class CommandType>
MinimalPathFollowingNode<CommandType>::MinimalPathFollowingNode(const rclcpp::NodeOptions & options)
: node_(std::make_unique<rclcpp::Node>("fira_minimal_node", options))
{

  declare_command_interface_configuration(node_, "cmd_output");
  declare_parameter_with_default(node_, "joy_start_button", XBOX_X_BUTTON);
  declare_parameter_with_default(node_, "joy_stop_button", XBOX_B_BUTTON);
  declare_parameter_with_default(node_, "setpoint.desired_linear_speed", 0.5);
  declare_parameter_with_default(node_, "setpoint.desired_lateral_deviation", 0.);
  declare_parameter_with_default(node_, "setpoint.desired_course_deviation", 0.);

  node_->get_parameter("joy_start_button", joy_start_button_id_);
  node_->get_parameter("joy_stop_button", joy_stop_button_id_);
  node_->get_parameter("setpoint.desired_linear_speed", setpoint_.linearSpeed);
  node_->get_parameter("setpoint.desired_lateral_deviation", setpoint_.lateralDeviation);
  node_->get_parameter("setpoint.desired_course_deviation", setpoint_.courseDeviation);

  auto interface_config = get_command_interface_configuration(node_, "cmd_output");
  cmd_interface_ = std::make_unique<VehicleInterface>(node_, std::move(interface_config));

  logger_ = std::make_shared<core::SimpleFileLogger>("/tmp/path_following.csv");
  path_following_ = make_path_following<CommandType>(node_, logger_);

  using namespace std::placeholders;
  auto matching_cb = std::bind(&MinimalPathFollowingNode::process_matching_info_, this, _1);
  matching_sub_ = node_->create_subscription<romea_path_msgs::msg::PathMatchingInfo2D>(
    "path_matching/info", reliable(1), std::move(matching_cb));

  auto odom_cb = std::bind(&MinimalPathFollowingNode::process_odometry_, this, _1);
  odometry_sub_ = node_->create_subscription<OdometryMeasureMsg>(
    "odometry", reliable(1), std::move(odom_cb));

  auto joystick_cb = std::bind(&MinimalPathFollowingNode::process_joystick_, this, _1);
  joystick_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
    "joy", reliable(1), std::move(joystick_cb));

  cmd_interface_->start();
}


//-----------------------------------------------------------------------------
template<class CommandType>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MinimalPathFollowingNode<CommandType>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}


//-----------------------------------------------------------------------------
template<typename CommandType>
void MinimalPathFollowingNode<CommandType>::process_odometry_(const OdometryMeasureMsg & msg)
{
  odometry_measure_.store(to_romea(msg.measure));
}


//-----------------------------------------------------------------------------
template<class CommandType>
void MinimalPathFollowingNode<CommandType>::process_matching_info_(
  romea_path_msgs::msg::PathMatchingInfo2D::ConstSharedPtr msg)
{
  // std::cout << "processMatchingInfo" << std::endl;
  // std::cout << *msg << std::endl;

  core::Twist2D filtered_twist = to_romea(msg->twist);
  std::vector<core::PathMatchedPoint2D> matchedPoints = to_romea(msg->matched_points);

  if (cmd_interface_->is_started()) {
    CommandType command = path_following_->computeCommand(
      setpoint_, matchedPoints, odometry_measure_.load(), filtered_twist);

    cmd_interface_->send_command(command);
    logger_->writeRow();
  }

}

//-----------------------------------------------------------------------------
template<class CommandType>
void MinimalPathFollowingNode<CommandType>::process_joystick_(
  sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  if (msg->buttons[joy_start_button_id_]) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("joystick"), "start button");
    path_following_->reset();
    cmd_interface_->start();
  }

  if (msg->buttons[joy_stop_button_id_]) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("joystick"), "stop button");
    cmd_interface_->stop(true);
  }
}


template class MinimalPathFollowingNode<core::TwoAxleSteeringCommand>;
template class MinimalPathFollowingNode<core::OneAxleSteeringCommand>;

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::TwoAxleSteeringMinimalPathFollowingNode)
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::OneAxleSteeringMinimalPathFollowingNode)
