cmake_minimum_required(VERSION 3.16)
project(course_project_autopilot LANGUAGES C CXX)

find_library(CPA_GPIOD_LIB gpiod REQUIRED)

add_library(cpa_mission_state src/domain/mission_state.cpp)
target_include_directories(cpa_mission_state PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(cpa_mission_state PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_drone_link_adapter src/io/drone_link_adapter.cpp)
target_include_directories(cpa_drone_link_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cpa_drone_link_adapter PUBLIC cpa_mission_state)
target_compile_options(cpa_drone_link_adapter PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_udp_socket src/io/udp_socket.cpp)
target_include_directories(cpa_udp_socket PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(cpa_udp_socket PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_gpio_controller src/io/gpio_controller.cpp)
target_include_directories(cpa_gpio_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cpa_gpio_controller PUBLIC ${CPA_GPIOD_LIB})
target_compile_options(cpa_gpio_controller PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_json_config_loader src/providers/JsonConfigLoader.cpp)
target_include_directories(cpa_json_config_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(cpa_json_config_loader PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_table_solver src/providers/TableSolver.cpp)
target_include_directories(cpa_table_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(cpa_table_solver PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_analytical_solver src/providers/AnalyticalSolver.cpp)
target_include_directories(cpa_analytical_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(cpa_analytical_solver PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_math_utils src/utils/math_utils.cpp)
target_include_directories(cpa_math_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(cpa_math_utils PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_simulation_recorder src/simulation/SimulationRecorder.cpp)
target_include_directories(cpa_simulation_recorder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(cpa_simulation_recorder PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_result_publisher src/publishing/ResultPublisher.cpp)
target_include_directories(cpa_result_publisher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(cpa_result_publisher PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_states
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp
    src/states/StateAccelerating.cpp
    src/states/StateMoving.cpp
    src/states/StateDecelerating.cpp)
target_include_directories(cpa_states PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cpa_states PUBLIC cpa_math_utils)
target_compile_options(cpa_states PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_mission_processor src/core/MissionProcessor.cpp)
target_include_directories(cpa_mission_processor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cpa_mission_processor PUBLIC cpa_math_utils cpa_states)
target_compile_options(cpa_mission_processor PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_target_analyzer src/core/TargetAnalyzer.cpp)
target_include_directories(cpa_target_analyzer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cpa_target_analyzer PUBLIC cpa_math_utils cpa_mission_state)
target_compile_options(cpa_target_analyzer PRIVATE -Wall -Wextra -pedantic)

target_link_libraries(cpa_mission_processor PUBLIC cpa_target_analyzer)

add_library(cpa_mavlink_gateway src/mavlink/mavlink_gateway.cpp)
target_include_directories(cpa_mavlink_gateway PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(cpa_mavlink_gateway SYSTEM PUBLIC ${CMAKE_SOURCE_DIR}/c_library_v2)
target_link_libraries(cpa_mavlink_gateway PUBLIC cpa_mission_state cpa_udp_socket)
target_compile_options(cpa_mavlink_gateway PRIVATE -Wall -Wextra -pedantic)

add_library(cpa_component_factory src/core/ComponentFactory.cpp)
target_include_directories(cpa_component_factory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cpa_component_factory PUBLIC cpa_drone_link_adapter cpa_gpio_controller cpa_json_config_loader cpa_table_solver cpa_analytical_solver cpa_mission_processor)
target_compile_options(cpa_component_factory PRIVATE -Wall -Wextra -pedantic)

add_executable(course_project_autopilot_app src/main.cpp)
target_include_directories(course_project_autopilot_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(course_project_autopilot_app PRIVATE cpa_component_factory cpa_drone_link_adapter cpa_gpio_controller cpa_json_config_loader cpa_table_solver cpa_mission_processor cpa_simulation_recorder cpa_result_publisher cpa_mavlink_gateway)
target_compile_definitions(course_project_autopilot_app PRIVATE DEFAULT_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(course_project_autopilot_app PRIVATE -Wall -Wextra -pedantic)
