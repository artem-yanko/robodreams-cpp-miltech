add_library(analytical_solver src/providers/AnalyticalSolver.cpp)
target_include_directories(analytical_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(analytical_solver PRIVATE -Wall -Wextra -pedantic)

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

add_library(mission_processor src/core/MissionProcessor.cpp)
target_include_directories(mission_processor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(mission_processor PRIVATE -Wall -Wextra -pedantic)

add_library(component_factory src/core/ComponentFactory.cpp)
target_include_directories(component_factory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(component_factory PUBLIC analytical_solver json_config_loader json_target_provider mission_processor target_analyzer math_utils)
target_compile_options(component_factory PRIVATE -Wall -Wextra -pedantic)

add_executable(mission_runner_app src/main.cpp)
target_include_directories(mission_runner_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mission_runner_app PRIVATE component_factory)
target_compile_options(mission_runner_app PRIVATE -Wall -Wextra -pedantic)
