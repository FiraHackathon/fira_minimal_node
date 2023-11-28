cmake_minimum_required(VERSION 3.8)
project(fira_minimal_node)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(romea_core_control REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_core_path REQUIRED)
find_package(romea_core_path_following REQUIRED)
find_package(romea_mobile_base_utils REQUIRED)
find_package(romea_mobile_base_msgs REQUIRED)
find_package(romea_path_utils REQUIRED)
find_package(romea_path_msgs REQUIRED)
find_package(romea_following_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/minimal_path_following_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  romea_core_mobile_base
  romea_core_control
  romea_core_path
  romea_core_path_following
  romea_mobile_base_utils
  romea_mobile_base_msgs
  romea_path_utils
  romea_path_msgs
  romea_core_control
  romea_following_utils
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::ros2::TwoAxleSteeringMinimalPathFollowingNode"
  EXECUTABLE two_axle_steering_minimal_path_following_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::ros2::OneAxleSteeringMinimalPathFollowingNode"
  EXECUTABLE one_axle_steering_minimal_path_following_node
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
# install(TARGETS
#   ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()
