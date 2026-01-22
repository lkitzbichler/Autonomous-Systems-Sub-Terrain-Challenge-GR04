cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

project(simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# --- External projects ------------
include(ExternalProject)
include(GNUInstallDirs)  # gives CMAKE_INSTALL_{BINDIR,LIBDIR,…}

# libsocket external dependency (installed into current install prefix)
ExternalProject_Add(libsocket
  GIT_REPOSITORY https://github.com/dermesser/libsocket
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
             -DCMAKE_BUILD_TYPE:STRING=Release
             -DBUILD_SHARED_LIBS=ON
             -DCMAKE_POLICY_DEFAULT_CMP0077=NEW
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install
)

# Where ExternalProject will install headers and libs
set(LIBSOCKET_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(LIBSOCKET_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

# --- Includes / subdirs ------------------------------------------------------
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${LIBSOCKET_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/tcpimage
)

add_subdirectory(tcpimage)

# --- Executables --------------------------------------------------------------
add_executable(w_to_unity src/w_to_unity.cpp)
ament_target_dependencies(w_to_unity
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  mav_msgs
)

find_package(Threads REQUIRED)
target_link_libraries(w_to_unity socket++ Threads::Threads)
target_link_directories(w_to_unity PRIVATE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
add_dependencies(w_to_unity libsocket)

add_executable(state_estimate_corruptor_node src/state_estimate_corruptor_node.cpp )
ament_target_dependencies(state_estimate_corruptor_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(state_estimate_corruptor_node)

add_executable(unity_ros src/unity_ros.cpp)
ament_target_dependencies(unity_ros
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  tf2
  tf2_ros
)
target_link_libraries(unity_ros tcpimage)
target_link_directories(unity_ros PRIVATE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
add_dependencies(unity_ros libsocket)

add_executable(unity_state src/unity_state.cpp)
ament_target_dependencies(unity_state
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(unity_state tcpstreamreader)
target_link_directories(unity_state PRIVATE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
add_dependencies(unity_state libsocket)

# --- Install targets -------------------------------------
install(TARGETS
  w_to_unity
  state_estimate_corruptor_node
  unity_ros
  unity_state
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install all launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Export and package ------------------------------------------------------
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
)

ament_package()
