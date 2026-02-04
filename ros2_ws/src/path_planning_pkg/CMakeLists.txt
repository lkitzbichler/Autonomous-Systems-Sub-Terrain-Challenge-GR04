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
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL REQUIRED ompl)

add_executable(pathplanner_node src/pathplanner.cpp)
ament_target_dependencies(
  pathplanner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  octomap
  octomap_msgs
  mav_planning_msgs
  mav_trajectory_generation
)
target_link_libraries(pathplanner_node Eigen3::Eigen)
target_link_libraries(pathplanner_node ${OCTOMAP_LIBRARIES})
target_link_libraries(pathplanner_node ${OMPL_LIBRARIES})
target_include_directories(pathplanner_node SYSTEM PUBLIC ${OMPL_INCLUDE_DIRS})


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

install(TARGETS pathplanner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
