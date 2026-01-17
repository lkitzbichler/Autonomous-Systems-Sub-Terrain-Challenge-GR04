# Install script for directory: /home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/src/simulation

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/install/simulation")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/w_to_unity" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/w_to_unity")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/w_to_unity"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/simulation" TYPE EXECUTABLE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/w_to_unity")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/w_to_unity" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/w_to_unity")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/w_to_unity"
         OLD_RPATH "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/install/simulation/lib:/opt/ros/jazzy/lib:/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/install/mav_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/w_to_unity")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/state_estimate_corruptor_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/state_estimate_corruptor_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/state_estimate_corruptor_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/simulation" TYPE EXECUTABLE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/state_estimate_corruptor_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/state_estimate_corruptor_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/state_estimate_corruptor_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/state_estimate_corruptor_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/state_estimate_corruptor_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/unity_ros" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/unity_ros")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/unity_ros"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/simulation" TYPE EXECUTABLE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/unity_ros")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/unity_ros" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/unity_ros")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/unity_ros"
         OLD_RPATH "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/install/simulation/lib:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/simulation/unity_ros")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation" TYPE DIRECTORY FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/src/simulation/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/simulation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/simulation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation/environment" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation/environment" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_index/share/ament_index/resource_index/packages/simulation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation/cmake" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation/cmake" TYPE FILE FILES
    "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_core/simulationConfig.cmake"
    "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/ament_cmake_core/simulationConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simulation" TYPE FILE FILES "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/src/simulation/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/tcpimage/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
