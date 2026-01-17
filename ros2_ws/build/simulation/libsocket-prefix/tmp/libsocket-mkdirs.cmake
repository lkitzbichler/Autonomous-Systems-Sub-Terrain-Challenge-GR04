# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src/libsocket"
  "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src/libsocket-build"
  "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix"
  "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/tmp"
  "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src/libsocket-stamp"
  "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src"
  "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src/libsocket-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src/libsocket-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src/libsocket-stamp${cfgdir}") # cfgdir has leading slash
endif()
