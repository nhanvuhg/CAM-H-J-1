cmake_minimum_required(VERSION 3.5)
project(com_rs485)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rs485_bus_node src/rs485_bus_node.cpp)
ament_target_dependencies(rs485_bus_node rclcpp std_msgs)
target_link_libraries(rs485_bus_node modbus)

install(TARGETS
  rs485_bus_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
