cmake_minimum_required(VERSION 3.8)
project(path_planning_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(frontier_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pathplanner_node src/pathplanner.cpp)
ament_target_dependencies(pathplanner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  trajectory_msgs
  octomap_msgs
  tf2_ros
  tf2_geometry_msgs)

if(TARGET ompl::ompl)
  target_link_libraries(pathplanner_node ompl::ompl)
elseif(OMPL_LIBRARIES)
  target_link_libraries(pathplanner_node ${OMPL_LIBRARIES})
  target_include_directories(pathplanner_node PRIVATE ${OMPL_INCLUDE_DIRS})
else()
  target_link_libraries(pathplanner_node ompl)
endif()

if(OCTOMAP_LIBRARIES)
  target_link_libraries(pathplanner_node ${OCTOMAP_LIBRARIES})
  target_include_directories(pathplanner_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
endif()

# Octomap -> 2D slice converter
add_executable(octomap_slice_node src/octomap_slice_node.cpp)
ament_target_dependencies(octomap_slice_node
  rclcpp
  std_msgs
  nav_msgs
  octomap_msgs)

if(OCTOMAP_LIBRARIES)
  target_link_libraries(octomap_slice_node ${OCTOMAP_LIBRARIES})
  target_include_directories(octomap_slice_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
endif()

# Exploration node with frontier integration and multi-slice logic
add_executable(exploration_node 
  src/exploration_node.cpp
  src/multi_slice_logic.cpp
  src/frontier_to_ompl.cpp)

target_include_directories(exploration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(exploration_node 
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  frontier_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS pathplanner_node octomap_slice_node exploration_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
