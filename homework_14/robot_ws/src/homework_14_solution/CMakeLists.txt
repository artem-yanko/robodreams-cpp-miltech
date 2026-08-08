cmake_minimum_required(VERSION 3.20)
project(homework_14_solution)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(underground_world REQUIRED)

add_executable(mission_explorer src/mission_explorer.cpp)
ament_target_dependencies(mission_explorer rclcpp underground_world)

add_executable(payload_action src/payload_action.cpp)
ament_target_dependencies(payload_action rclcpp underground_world)

install(TARGETS
  mission_explorer
  payload_action
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
