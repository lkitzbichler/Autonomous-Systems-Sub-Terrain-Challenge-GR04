cmake_minimum_required(VERSION 3.8)
project(basic_waypoint_pkg)

# ==== General build options ====
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==== Dependencies ====
find_package(ament_cmake REQUIRED)

# Assumes you ported the node to rclcpp (ros::NodeHandle → rclcpp::Node, etc.)
find_package(rclcpp REQUIRED)

# ROS2 versions of your mav-related packages
find_package(mav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(glog REQUIRED)

# Replaces eigen_catkin
find_package(Eigen3 REQUIRED)

# Replaces yaml_cpp_catkin
find_package(yaml-cpp REQUIRED)

# ==== Executable ====
# You can keep the original target name or pick something clearer like planner_node.
add_executable(basic_waypoint_node
  src/planner.cc
  src/planner_node.cc
)

target_include_directories(basic_waypoint_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)


# Pull in include dirs + link flags for all ament packages
ament_target_dependencies(basic_waypoint_node
  rclcpp
  mav_msgs
  std_msgs
  mav_planning_msgs
  mav_trajectory_generation
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  glog
  Eigen3
  yaml-cpp
)

# ==== Installation ====
install(
  TARGETS
    basic_waypoint_node
  DESTINATION lib/${PROJECT_NAME}
)


# If you have headers under include/basic_waypoint_pkg/...
install(
  DIRECTORY include/
  DESTINATION include
)

# Install all launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ==== Export ====
ament_package()
