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
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Qml Quick Charts QuickControls2 Widgets Network LinguistTools)

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
  include/unified_control_gui/system_alert_controller.hpp
  include/unified_control_gui/language_controller.hpp
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
  src/system_alert_controller.cpp
  src/language_controller.cpp
)

set(TS_FILES
  translations/unified_control_gui_core_vi.ts
  translations/unified_control_gui_camera_vi.ts
  translations/unified_control_gui_cartridge_vi.ts
  translations/unified_control_gui_cpp_vi.ts
)
qt5_add_translation(QM_FILES ${TS_FILES})

qt5_add_resources(QML_RESOURCES qml.qrc)

add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADER_FILES}
  ${QML_RESOURCES}
  ${QM_FILES}
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
  ament_index_cpp
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

install(FILES ${QM_FILES}
  DESTINATION share/${PROJECT_NAME}/i18n
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_i18n_catalog tests/test_i18n_catalog.py)
  ament_add_pytest_test(test_restart_contract tests/test_restart_contract.py)
endif()

ament_package()
