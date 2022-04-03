cmake_minimum_required(VERSION 3.5)
project(emergency_stop)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(action_msgs REQUIRED)
#boost
find_package(Boost REQUIRED COMPONENTS system)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(emergency_stop_node 
  src/emergency_stop_node.cpp 
  src/emergency_stop.cpp
)
target_include_directories(emergency_stop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  emergency_stop_node
  rclcpp
  sensor_msgs
  std_msgs
  Boost
)

install(TARGETS emergency_stop_node
  DESTINATION lib/${PROJECT_NAME})

install(
DIRECTORY include/
DESTINATION include
)


# Need these 2 export macros so other packages can find header files!
ament_export_include_directories(
  include
  )

ament_export_libraries(
  ${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
