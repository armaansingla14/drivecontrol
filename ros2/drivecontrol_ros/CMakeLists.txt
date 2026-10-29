cmake_minimum_required(VERSION 3.16)
project(drivecontrol_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# the controller library from the repository root
set(DC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
add_library(drivecontrol STATIC
  ${DC_ROOT}/src/common.cpp
  ${DC_ROOT}/src/path.cpp
  ${DC_ROOT}/src/lateral.cpp
  ${DC_ROOT}/src/longitudinal.cpp
  ${DC_ROOT}/src/can_interface.cpp
  ${DC_ROOT}/src/safety.cpp
  ${DC_ROOT}/src/controller.cpp)
target_include_directories(drivecontrol PUBLIC ${DC_ROOT}/include)
set_target_properties(drivecontrol PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(drivecontrol_node src/drivecontrol_node.cpp)
target_link_libraries(drivecontrol_node drivecontrol)
ament_target_dependencies(drivecontrol_node rclcpp nav_msgs std_msgs geometry_msgs ackermann_msgs)

install(TARGETS drivecontrol_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
