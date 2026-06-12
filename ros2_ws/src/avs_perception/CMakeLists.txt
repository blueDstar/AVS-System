cmake_minimum_required(VERSION 3.8)
project(avs_perception)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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
ament_target_dependencies(ipm_transform_node
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
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
