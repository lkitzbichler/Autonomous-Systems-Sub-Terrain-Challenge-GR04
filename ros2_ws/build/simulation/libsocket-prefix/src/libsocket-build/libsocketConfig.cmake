set(libsocket_INCLUDE_DIRS "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src/libsocket/headers")

set(libsocket_BINARY_DIR "/home/leon/Dev/TUM/autonome-systeme/project/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws/build/simulation/libsocket-prefix/src/libsocket-build")

include(${libsocket_BINARY_DIR}/libsocketTargets.cmake)

set(libsocket_LIBRARIES socket++)
