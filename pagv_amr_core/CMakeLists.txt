cmake_minimum_required(VERSION 3.8)
project(pagv_amr_core)

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
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Include directories
include_directories(
  include
  #third_party
  third_party/yaml-cpp/include
  third_party/libvda5050pp/include/public/vda5050++
  third_party/libvda5050pp/include/private
  third_party/rapidjson/include  
)

add_subdirectory(third_party/yaml-cpp)

add_executable(amr_core_node
  # Main
  src/amr_core_node.cpp
  
  # VDA5050 Protocol 
  src/vda5050_protocol.cpp
  
  # Order & Navigation
  src/order_execution/order_execution_manager.cpp
  src/navigation/navigation_manager.cpp
  src/navigation/path_follower.cpp
  src/navigation/arrival_detector.cpp
  
  # BT Nodes
  src/bt_nodes/mode_layer/mode_layer_node.cpp
  src/bt_nodes/safety_layer/safety_layer_node.cpp
  src/bt_nodes/system_layer/system_layer_node.cpp
  src/bt_nodes/mission_layer/mission_layer_node.cpp
  src/bt_nodes/navigation_layer/navigation_layer_node.cpp
)

target_link_libraries(amr_core_node 
    yaml-cpp  
    behaviortree_cpp::behaviortree_cpp
    # paho-mqttpp3
    # paho-mqtt3as   
)


ament_target_dependencies(amr_core_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  behaviortree_cpp
)

# Link Paho MQTT libraries (VDA5050 Protocol용)
find_package(PahoMqttCpp QUIET)
if(PahoMqttCpp_FOUND)
  target_link_libraries(amr_core_node PahoMqttCpp::paho-mqttpp3)
else()
  target_link_libraries(amr_core_node paho-mqttpp3 paho-mqtt3as)
endif()

# Install
install(TARGETS
  amr_core_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
  OPTIONAL
)

ament_package()
