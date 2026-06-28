cmake_minimum_required(VERSION 3.16)
project(homework_11 LANGUAGES C CXX)

find_library(HW11_GPIOD_LIB gpiod REQUIRED)

add_library(hw11_mission_state src/domain/mission_state.cpp)
target_include_directories(hw11_mission_state PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw11_mission_state PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_drone_link_adapter src/io/drone_link_adapter.cpp)
target_include_directories(hw11_drone_link_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hw11_drone_link_adapter PUBLIC hw11_mission_state)
target_compile_options(hw11_drone_link_adapter PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_gpio_controller src/io/gpio_controller.cpp)
target_include_directories(hw11_gpio_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hw11_gpio_controller PUBLIC ${HW11_GPIOD_LIB})
target_compile_options(hw11_gpio_controller PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_json_config_loader src/providers/JsonConfigLoader.cpp)
target_include_directories(hw11_json_config_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw11_json_config_loader PRIVATE -Wall -Wextra -pedantic)

add_executable(homework_11_app src/main.cpp)
target_include_directories(homework_11_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(homework_11_app PRIVATE hw11_drone_link_adapter hw11_gpio_controller hw11_json_config_loader)
target_compile_definitions(homework_11_app PRIVATE DEFAULT_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(homework_11_app PRIVATE -Wall -Wextra -pedantic)
