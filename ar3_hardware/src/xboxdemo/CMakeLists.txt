cmake_minimum_required(VERSION 3.10.2)
project(xboxdemo)
#set(planning_scene_monitor_DIR )
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

set(SERVO_CONTROLLER_INPUT servo_controller_input)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  control_toolbox
  moveit_msgs
  geometry_msgs
  tf2_eigen
  trajectory_msgs
  moveit_ros_planning
)

# Common cmake code applied to all moveit packages
# find_package(moveit_common REQUIRED)
# moveit_package()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  moveit_servo
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

include_directories(
    include
)

add_executable(servo_cpp_interface_demo src/servo_cpp_interface_demo.cpp)
target_include_directories(servo_cpp_interface_demo PUBLIC include)
ament_target_dependencies(servo_cpp_interface_demo ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)

install (
  TARGETS
    servo_cpp_interface_demo

  DESTINATION
    lib/${PROJECT_NAME}
)

install(
DIRECTORY include 
DESTINATION share/${PROJECT_NAME}
)

install(
DIRECTORY launch 
DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()


ament_package()
