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

#ifndef ROMEA_PATH_FOLLOWING__FACTORY__PATH_FOLLOWING_COMMAND_FACTORY_HPP_
#define ROMEA_PATH_FOLLOWING__FACTORY__PATH_FOLLOWING_COMMAND_FACTORY_HPP_

// std
#include <memory>
#include <string>

#include "fira_minimal_node/base_parameters.hpp"
#include "fira_minimal_node/command_parameters.hpp"

namespace romea
{
namespace ros2
{

template<typename PathSectionFollowing, typename Node>
std::unique_ptr<PathSectionFollowing> make_path_section_following_impl(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  using CommandLimits = typename PathSectionFollowing::CommandLimits; // NOLINT
  declare_longitudinal_control_parameters(node);
  declare_lateral_control_parameters<PathSectionFollowing>(node, params_ns);
  declare_command_limits<typename PathSectionFollowing::CommandLimits>(node);

  return std::make_unique<PathSectionFollowing>(
    get_wheelbase(node),
    get_inertia(node),
    get_longitudinal_control_parameters(node),
    get_lateral_control_parameters<PathSectionFollowing>(node, params_ns),
    get_command_limits<CommandLimits>(node));
}


template<typename CommandType, typename Node>
std::unique_ptr<core::PathSectionFollowingBase<CommandType>>
make_path_section_following(
  std::shared_ptr<Node> node,
  const std::string & lateral_control_params_ns,
  const std::string & lateral_control_type)
{
  if (lateral_control_type == "classic") {
    using Control = core::PathSectionFollowingClassic<CommandType>; // NOLINT
    return make_path_section_following_impl<Control>(node, lateral_control_params_ns);
  } else if (lateral_control_type == "predictive") {
    using Control = core::PathSectionFollowingPredictive<CommandType>; // NOLINT
    return make_path_section_following_impl<Control>(node, lateral_control_params_ns);
  } else {
    throw std::runtime_error(
            "Configuration for " + lateral_control_type + " path section following not found");
  }
}

template<typename CommandType, typename Node>
std::unique_ptr<core::PathSectionFollowingBase<CommandType>>
make_path_section_following(std::shared_ptr<Node> node, const std::string & ns)
{
  declare_lateral_control_type(node, ns);
  auto lateral_control_type = get_lateral_control_type(node, ns);

  if (lateral_control_type != "none") {
    return make_path_section_following<CommandType>(
      node, ns + ".configuration", lateral_control_type);
  } else {
    return {};
  }
}

template<typename CommandType, typename Node>
std::unique_ptr<core::PathSectionFollowingBase<CommandType>>
make_path_section_following_from_list(std::shared_ptr<Node> node, const std::string & ns)
{
  declare_selected_lateral_control_type(node, ns);
  auto lateral_control_type = get_selected_lateral_control_type(node, ns);

  if (lateral_control_type != "none") {
    return make_path_section_following<CommandType>(
      node, ns + "." + lateral_control_type, lateral_control_type);
  } else {
    return {};
  }
}

template<typename CommandType, typename Node>
std::unique_ptr<core::PathFollowingBase<CommandType>>
make_path_following(
  std::shared_ptr<Node> node,
  std::shared_ptr<core::SimpleFileLogger> logger = nullptr)
{
  declare_inertia(node);
  declare_wheelbase(node);
  auto path_section_following = make_path_section_following_from_list<CommandType>(
    node, "lateral_controls");

  if constexpr (std::is_same_v<CommandType, core::OneAxleSteeringCommand>) {
    return std::make_unique<core::OneAxleSteeringPathFollowing>(
      std::move(path_section_following), logger);
  } else if constexpr (std::is_same_v<CommandType, core::TwoAxleSteeringCommand>) {
    return std::make_unique<core::TwoAxleSteeringPathFollowing>(
      std::move(path_section_following), logger);
  } else {
    return nullptr;
  }
}


}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__FACTORY__PATH_FOLLOWING_COMMAND_FACTORY_HPP_
