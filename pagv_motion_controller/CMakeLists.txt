cmake_minimum_required(VERSION 3.8)
project(pagv_motion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Motion Controller Node
add_executable(motion_controller_node
  src/motion_controller_node.cpp
)

ament_target_dependencies(motion_controller_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install
install(TARGETS
  motion_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
