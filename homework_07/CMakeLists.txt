add_library(analytical_solver src/providers/AnalyticalSolver.cpp)
target_include_directories(analytical_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(analytical_solver PRIVATE -Wall -Wextra -pedantic)

add_library(table_solver src/providers/TableSolver.cpp)
target_include_directories(table_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(table_solver PRIVATE -Wall -Wextra -pedantic)

add_library(json_config_loader src/providers/JsonConfigLoader.cpp)
target_include_directories(json_config_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(json_config_loader PRIVATE -Wall -Wextra -pedantic)

add_library(json_target_provider src/providers/JsonTargetProvider.cpp)
target_include_directories(json_target_provider PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(json_target_provider PRIVATE -Wall -Wextra -pedantic)

add_library(math_utils src/utils/math_utils.cpp)
target_include_directories(math_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(math_utils PRIVATE -Wall -Wextra -pedantic)

add_library(target_analyzer src/core/TargetAnalyzer.cpp)
target_include_directories(target_analyzer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(target_analyzer PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(target_analyzer PUBLIC math_utils)

add_library(mission_processor src/core/MissionProcessor.cpp)
target_include_directories(mission_processor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(mission_processor PRIVATE -Wall -Wextra -pedantic)

add_library(state_stopped src/states/StateStopped.cpp)
target_include_directories(state_stopped PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(state_stopped PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(state_stopped PUBLIC math_utils state_turning state_accelerating)

add_library(state_turning src/states/StateTurning.cpp)
target_include_directories(state_turning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(state_turning PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(state_turning PUBLIC math_utils state_accelerating)

add_library(state_accelerating src/states/StateAccelerating.cpp)
target_include_directories(state_accelerating PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(state_accelerating PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(state_accelerating PUBLIC math_utils state_moving)

add_library(state_moving src/states/StateMoving.cpp)
target_include_directories(state_moving PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(state_moving PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(state_moving PUBLIC math_utils state_decelerating)

add_library(state_decelerating src/states/StateDecelerating.cpp)
target_include_directories(state_decelerating PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(state_decelerating PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(state_decelerating PUBLIC math_utils state_turning state_accelerating)

add_library(drone_physics src/core/DronePhysics.cpp)
target_include_directories(drone_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(drone_physics PRIVATE -Wall -Wextra -pedantic)

target_link_libraries(mission_processor PUBLIC target_analyzer math_utils drone_physics state_stopped state_turning state_accelerating state_moving state_decelerating)

add_library(component_factory src/core/ComponentFactory.cpp)
target_include_directories(component_factory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(component_factory PUBLIC analytical_solver table_solver json_config_loader json_target_provider mission_processor)
target_compile_options(component_factory PRIVATE -Wall -Wextra -pedantic)

add_executable(mission_runner_app src/main.cpp)
target_include_directories(mission_runner_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mission_runner_app PRIVATE component_factory)
target_compile_options(mission_runner_app PRIVATE -Wall -Wextra -pedantic)
