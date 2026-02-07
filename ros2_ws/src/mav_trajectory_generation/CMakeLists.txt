cmake_minimum_required(VERSION 3.8)
project(mav_trajectory_generation)

# set BUILD_TESTING to OFF
set (BUILD_TESTING OFF CACHE BOOL "Disable building tests" FORCE)

include(ExternalProject)
include(GNUInstallDirs)  # For CMAKE_INSTALL_LIBDIR etc.

# ==== General build options ====
# ROS2 typically uses at least C++14; adapt if you really need C++11.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==== Dependencies ====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)
# Pure C++ library; no ROS-specific deps needed here unless your code uses them.
# If you add rclcpp or others, list them here and in ament_target_dependencies.
find_package(yaml-cpp REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)

ExternalProject_Add(nlopt_ext
  GIT_REPOSITORY https://github.com/stevengj/nlopt.git
  GIT_TAG v2.7.1              # or another stable tag if you prefer
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DBUILD_SHARED_LIBS=ON
    # Turn off all language bindings and extras, we only want the C/C++ lib:
    -DNLOPT_PYTHON=OFF
    -DNLOPT_OCTAVE=OFF
    -DNLOPT_MATLAB=OFF
    -DNLOPT_GUILE=OFF
    -DNLOPT_SWIG=OFF
    -DNLOPT_MATHEMATICA=OFF
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install
)

set(NLOPT_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(NLOPT_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}
)
link_directories(
  ${NLOPT_LIBRARY_DIR}
)

# ==== Library ====
add_library(${PROJECT_NAME} SHARED
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
  src/ros_conversions.cpp
)

add_dependencies(${PROJECT_NAME} 
  nlopt_ext
)

target_link_libraries(${PROJECT_NAME}
    glog::glog
    nlopt
    yaml-cpp
)

# If you have public headers under include/mav_trajectory_generation
# adjust this to your actual include path.
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-checks/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  mav_msgs
  Eigen3
  mav_planning_msgs
  rclcpp
)


# ==== Executable ====
add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation
  ${PROJECT_NAME}
)

ament_target_dependencies(polynomial_timing_evaluation
  mav_msgs
  Eigen3
)

add_executable(trajectory_sampler_node
  src/trajectory_sampler_node.cpp
)
target_link_libraries(trajectory_sampler_node
  ${PROJECT_NAME}
)
ament_target_dependencies(trajectory_sampler_node
  mav_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  mav_planning_msgs
  Eigen3
  rclcpp
)

# ==== Tests ====
# Only if you actually want to build the tests when BUILD_TESTING is ON.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_polynomial
    test/test_polynomial.cpp
  )
  if(TARGET test_polynomial)
    target_link_libraries(test_polynomial
      ${PROJECT_NAME}
    )
  endif()

  ament_add_gtest(test_polynomial_optimization
    test/test_polynomial_optimization.cpp
  )
  if(TARGET test_polynomial_optimization)
    target_link_libraries(test_polynomial_optimization
      ${PROJECT_NAME}
    )
  endif()
endif()

# ==== Installation ====
install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    polynomial_timing_evaluation
    trajectory_sampler_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers if you have them.
# Assumes headers live in include/mav_trajectory_generation/...
install(
  DIRECTORY include/
  DESTINATION include
)

# install nlopt libraries
install(DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  FILES_MATCHING PATTERN "libnlopt.*"
)

# ==== Export for downstream packages ====
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(yaml-cpp)
ament_export_dependencies(glog)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
