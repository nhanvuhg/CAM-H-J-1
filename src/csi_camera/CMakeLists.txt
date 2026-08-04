cmake_minimum_required(VERSION 3.18)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc CACHE FILEPATH
    "Jetson CUDA compiler")
project(csi_camera LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

# Create the single production executable. The old GStreamer/Argus node is
# intentionally not part of this package anymore; production uses direct V4L2.
add_executable(v4l2_dual_camera_cuda_node
  src/v4l2_dual_camera_cuda_node.cu
)

target_include_directories(v4l2_dual_camera_cuda_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(v4l2_dual_camera_cuda_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(v4l2_dual_camera_cuda_node
  ${OpenCV_LIBS}
  CUDA::cudart
)

set_target_properties(v4l2_dual_camera_cuda_node PROPERTIES
  CUDA_ARCHITECTURES 87
  CUDA_SEPARABLE_COMPILATION ON
)

target_compile_options(v4l2_dual_camera_cuda_node PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math;-Xcompiler=-Wall,-Wextra,-Wno-unused-parameter>
)

# Install executables
install(TARGETS
  v4l2_dual_camera_cuda_node
  DESTINATION lib/${PROJECT_NAME}
)

# Stable V3Link backend used on this Jetson. It publishes both cameras from a
# single process so the shared V4L2/CSI path is time-multiplexed safely.
install(PROGRAMS
  scripts/v4l2_dual_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME v4l2_dual_camera_node
)

# Install only the Jetson production launcher. Legacy Raspberry Pi/Hailo
# examples live under docs/ and must never appear in `ros2 launch csi_camera`.
install(FILES
  launch/dual_camera_system.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
