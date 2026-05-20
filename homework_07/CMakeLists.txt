add_library(analytical_solver src/providers/AnalyticalSolver.cpp)
target_include_directories(analytical_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(analytical_solver PRIVATE -Wall -Wextra -pedantic)

add_library(json_config_loader src/providers/JsonConfigLoader.cpp)
target_include_directories(json_config_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(json_config_loader PRIVATE -Wall -Wextra -pedantic)

add_executable(mission_runner_app src/main.cpp)
target_include_directories(mission_runner_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mission_runner_app PRIVATE analytical_solver json_config_loader)
target_compile_options(mission_runner_app PRIVATE -Wall -Wextra -pedantic)