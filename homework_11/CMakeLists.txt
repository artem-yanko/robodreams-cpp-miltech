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

add_library(hw11_table_solver src/providers/TableSolver.cpp)
target_include_directories(hw11_table_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw11_table_solver PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_math_utils src/utils/math_utils.cpp)
target_include_directories(hw11_math_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw11_math_utils PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_states
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp
    src/states/StateAccelerating.cpp
    src/states/StateMoving.cpp
    src/states/StateDecelerating.cpp)
target_include_directories(hw11_states PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hw11_states PUBLIC hw11_math_utils)
target_compile_options(hw11_states PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_mission_processor src/core/MissionProcessor.cpp)
target_include_directories(hw11_mission_processor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hw11_mission_processor PUBLIC hw11_math_utils hw11_states)
target_compile_options(hw11_mission_processor PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_target_analyzer src/core/TargetAnalyzer.cpp)
target_include_directories(hw11_target_analyzer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hw11_target_analyzer PUBLIC hw11_math_utils hw11_mission_state)
target_compile_options(hw11_target_analyzer PRIVATE -Wall -Wextra -pedantic)

target_link_libraries(hw11_mission_processor PUBLIC hw11_target_analyzer)

add_executable(homework_11_app src/main.cpp)
target_include_directories(homework_11_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(homework_11_app PRIVATE hw11_drone_link_adapter hw11_gpio_controller hw11_json_config_loader hw11_table_solver hw11_mission_processor)
target_compile_definitions(homework_11_app PRIVATE DEFAULT_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(homework_11_app PRIVATE -Wall -Wextra -pedantic)
