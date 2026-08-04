cmake_minimum_required(VERSION 3.16)
project(yolo_tensorrt_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)
find_package(CUDAToolkit REQUIRED)

find_path(
  TENSORRT_INCLUDE_DIR
  NvInfer.h
  HINTS
    /usr/include/aarch64-linux-gnu
    /usr/include
)
find_library(
  TENSORRT_LIBRARY
  nvinfer
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib
)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR "TensorRT headers/library not found. Install libnvinfer-dev.")
endif()

add_executable(yolo_tensorrt_node src/yolo_tensorrt_node.cpp)
target_include_directories(
  yolo_tensorrt_node
  PRIVATE
    ${TENSORRT_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(
  yolo_tensorrt_node
  cv_bridge
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
)
target_link_libraries(
  yolo_tensorrt_node
  ${TENSORRT_LIBRARY}
  CUDA::cudart
  ${OpenCV_LIBS}
)

install(TARGETS yolo_tensorrt_node DESTINATION lib/${PROJECT_NAME})

ament_package()
