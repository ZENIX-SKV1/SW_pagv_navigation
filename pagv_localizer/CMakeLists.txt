cmake_minimum_required(VERSION 3.8)
project(pagv_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Localizer Node
add_executable(localizer_node
  src/localizer_node.cpp
)

ament_target_dependencies(localizer_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Install
install(TARGETS
  localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
