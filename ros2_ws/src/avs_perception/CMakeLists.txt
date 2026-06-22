cmake_minimum_required(VERSION 3.8)
project(avs_perception)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Build Type ────────────────────────────────────────────────────────────────
# Default to Release if not explicitly set (e.g. plain `colcon build`)
# This ensures GCC applies full -O3 optimizations instead of -O0 (Debug)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# ── CPU Optimizations (Release mode) ──────────────────────────────────────────
# Targets Raspberry Pi 5 (Cortex-A76, AArch64) at edge, and x86_64 on dev host.
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64" OR CMAKE_SYSTEM_NAME MATCHES "Darwin")
    add_compile_options(
      -O3
      -march=armv8.2-a
      -mtune=cortex-a76
      -ffast-math
      -funroll-loops
    )
    message(STATUS "AVS: Release build — ARM64 optimizations ENABLED (-O3 -march=armv8.2-a -mtune=cortex-a76)")
  else()
    add_compile_options(
      -O3
      -ffast-math
      -funroll-loops
    )
    message(STATUS "AVS: Release build — Generic/x86_64 optimizations ENABLED (-O3)")
  endif()
else()
  message(STATUS "AVS: Non-release build (${CMAKE_BUILD_TYPE}) — optimizations DISABLED")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Check for local NCNN build inside workspace or system paths
if(EXISTS "/home/goln/SimpleSysIDV/ncnn/lib/cmake/ncnn")
  set(ncnn_DIR "/home/goln/SimpleSysIDV/ncnn/lib/cmake/ncnn")
elseif(EXISTS "/usr/lib/cmake/ncnn")
  set(ncnn_DIR "/usr/lib/cmake/ncnn")
elseif(EXISTS "/usr/local/lib/cmake/ncnn")
  set(ncnn_DIR "/usr/local/lib/cmake/ncnn")
endif()
find_package(ncnn REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Fallback for NCNN include dirs
if(ncnn_FOUND)
  if(ncnn_INCLUDE_DIRS)
    include_directories(${ncnn_INCLUDE_DIRS})
  else()
    include_directories("/home/goln/SimpleSysIDV/ncnn/include")
  endif()
endif()

# Create library for the YOLO26 inference engine
add_library(yolo26_seg_lib src/yolo26_seg.cpp)
target_link_libraries(yolo26_seg_lib
  ncnn
  ${OpenCV_LIBRARIES}
)

# Create live ROS2 inference node
add_executable(ncnn_inference_node src/ncnn_inference_node.cpp)
target_link_libraries(ncnn_inference_node yolo26_seg_lib)
ament_target_dependencies(ncnn_inference_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

# Create offline video profiling node
add_executable(video_test_node src/video_test_node.cpp)
target_link_libraries(video_test_node yolo26_seg_lib)
ament_target_dependencies(video_test_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Create video publisher node for simulation
add_executable(video_publisher_node src/video_publisher_node.cpp)
target_link_libraries(video_publisher_node ${OpenCV_LIBRARIES})
ament_target_dependencies(video_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Create IPM coordinates transformation node
add_executable(ipm_transform_node src/ipm_transform_node.cpp)
target_link_libraries(ipm_transform_node ${OpenCV_LIBRARIES})
ament_target_dependencies(ipm_transform_node
  rclcpp
  std_msgs
  nav_msgs
)

# Create lane error publisher node
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node
  rclcpp
  std_msgs
)

# Install rules
install(TARGETS
  yolo26_seg_lib
  ncnn_inference_node
  video_test_node
  video_publisher_node
  ipm_transform_node
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
