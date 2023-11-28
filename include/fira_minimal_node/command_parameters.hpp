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

#ifndef ROMEA_PATH_FOLLOWING__PARAMS__PATH_FOLLOWING_COMMAND_PARAMETERS_HPP_
#define ROMEA_PATH_FOLLOWING__PARAMS__PATH_FOLLOWING_COMMAND_PARAMETERS_HPP_

// std
#include <memory>
#include <string>

// romea
#include "romea_core_path_following/command/PathSectionFollowingClassic.hpp"
#include "romea_core_path_following/command/PathSectionFollowingPredictive.hpp"
#include "romea_core_path_following/command/PathSectionFollowingLongitudinalControl.hpp"
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_following_utils/command_factory.hpp"

namespace romea
{
namespace ros2
{

template<typename Node>
void declare_lateral_control_type(std::shared_ptr<Node> node, const std::string & ns)
{
  declare_parameter_with_default<std::string>(node, ns, "type", "none");
}

template<typename Node>
std::string get_lateral_control_type(std::shared_ptr<Node> node, const std::string & ns)
{
  return get_parameter<std::string>(node, ns, "type");
}

template<typename Node>
void declare_selected_lateral_control_type(std::shared_ptr<Node> node, const std::string & ns)
{
  declare_parameter_with_default<std::string>(node, ns, "selected", "none");
}

template<typename Node>
std::string get_selected_lateral_control_type(std::shared_ptr<Node> node, const std::string & ns)
{
  return get_parameter<std::string>(node, ns, "selected");
}


template<typename Node>
void declare_longitudinal_control_parameters(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "longitudinal_control", "minimal_linear_speed");
}

template<typename Node>
core::PathSectionFollowingLongitudinalControlParameters
get_longitudinal_control_parameters(std::shared_ptr<Node> node)
{
  return {get_parameter<double>(node, "longitudinal_control", "minimal_linear_speed")};
}

template<typename PathFollowingSection, typename Node>
void declare_lateral_control_parameters(
  std::shared_ptr<Node> node,
  const std::string & ns)
{
  if constexpr (std::is_same_v<typename PathFollowingSection::LateralControl,
    core::FollowTrajectoryClassicSliding>)
  {
    declare_follow_trajectory_classic_sliding_parameters(node, ns);
  } else if constexpr (std::is_same_v<typename PathFollowingSection::LateralControl,
    core::FollowTrajectoryPredictiveSliding>)
  {
    declare_follow_trajectory_predictive_sliding_parameters(node, ns);
  }
}

template<typename PathFollowingSection, typename Node>
typename PathFollowingSection::LateralControlParameters
get_lateral_control_parameters(
  std::shared_ptr<Node> node,
  const std::string & ns)
{
  typename PathFollowingSection::LateralControlParameters parameters;
  get_params(node, ns, parameters);
  return parameters;
}

}  // namespace ros2
}  // namespace romea

#endif // ROMEA_PATH_FOLLOWING__PARAMS__PATH_FOLLOWING_COMMAND_PARAMETERS_HPP_
