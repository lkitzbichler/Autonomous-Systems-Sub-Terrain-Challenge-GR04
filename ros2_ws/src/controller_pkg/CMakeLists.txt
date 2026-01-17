cmake_minimum_required(VERSION 3.10)
project(controller_pkg)

# --- C++ standard (ROS 2 typically uses C++17) -------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Eigen
find_package(eigen3_cmake_module QUIET)
find_package(Eigen3 REQUIRED)

# --- Includes ----------------------------------------------------------------
include_directories(${EIGEN3_INCLUDE_DIRS})

# --- Libraries ----------------------------------------------------------------
add_library(controller_lib SHARED IMPORTED)
# Path to the precompiled .so inside the repository
set_target_properties(controller_lib PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libcontroller_lib.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# --- Executable ---------------------------------------------------------------
add_executable(controller_node src/controller_wrapper.cpp)

target_link_libraries(controller_node controller_lib)

ament_target_dependencies(controller_node
  rclcpp
  std_msgs
  geometry_msgs
  mav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_eigen
)

# --- Install -----------------------------------------------------------------
install(TARGETS controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcontroller_lib.so
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Export ------------------------------------------------------------------

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  Eigen3
)

ament_package()
