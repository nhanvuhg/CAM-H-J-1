cmake_minimum_required(VERSION 3.5)
project(unified_control_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dobot_msgs_v3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Qml Quick Charts QuickControls2 Widgets Network)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(HEADER_FILES
  include/unified_control_gui/cam_node.hpp
  include/unified_control_gui/cam_provider.hpp
  include/unified_control_gui/robot_controller.hpp
  include/unified_control_gui/cartridge_controller.hpp
  include/unified_control_gui/scale_controller.hpp
  include/unified_control_gui/hp_controller.hpp
  include/unified_control_gui/auth_controller.hpp
)

set(SOURCES
  src/main.cpp
  src/cam_node.cpp
  src/cam_provider.cpp
  src/robot_controller.cpp
  src/cartridge_controller.cpp
  src/scale_controller.cpp
  src/hp_controller.cpp
  src/auth_controller.cpp
)

qt5_add_resources(QML_RESOURCES qml.qrc)

add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADER_FILES}
  ${QML_RESOURCES}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Qml
  Qt5::Quick
  Qt5::Charts
  Qt5::QuickControls2
  Qt5::Widgets
  Qt5::Network
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  dobot_msgs_v3
  cv_bridge
  image_transport
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vfd_logic_node.py
  scripts/restart_system_nodes.sh
  scripts/restart_gui.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
