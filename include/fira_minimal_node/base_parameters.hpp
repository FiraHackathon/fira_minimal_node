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

#ifndef ROMEA_PATH_FOLLOWING__PARAMS__PATH_FOLLOWING_BASE_PARAMETERS_HPP_
#define ROMEA_PATH_FOLLOWING__PARAMS__PATH_FOLLOWING_BASE_PARAMETERS_HPP_

// std
#include <memory>

// romea
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"
#include "romea_mobile_base_utils/params/command_limits_parameters.hpp"
#include "romea_core_path_following/PathFollowingTraits.hpp"

namespace romea
{
namespace ros2
{

template<typename Node>
void declare_wheelbase(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "base", "wheelbase");
}

template<typename Node>
double get_wheelbase(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "base", "wheelbase");
}

template<typename Node>
void declare_inertia(std::shared_ptr<Node> node)
{
  declare_inertia_info(node, "base.inertia");
}

template<typename Node>
core::MobileBaseInertia get_inertia(std::shared_ptr<Node> node)
{
  return get_inertia_info(node, "base.inertia");
}

template<typename CommandLimits, typename Node>
void declare_command_limits(std::shared_ptr<Node> node)
{
  declare_command_limits<CommandLimits>(node, "base.command_limits");
}

template<typename CommandLimits, typename Node>
CommandLimits get_command_limits(std::shared_ptr<Node> node)
{
  return get_command_limits<CommandLimits>(node, "base.command_limits");
}

template<typename Node>
void declare_base_info(std::shared_ptr<Node> node)
{
  declare_wheelbase(node);
  declare_inertia_info(node);
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PARAMS__PATH_FOLLOWING_BASE_PARAMETERS_HPP_
