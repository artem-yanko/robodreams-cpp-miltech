add_library(ballistics src/ballistics.cpp)
target_include_directories(ballistics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(ballistics_check src/main.cpp)
target_link_libraries(ballistics_check PRIVATE ballistics)
target_compile_options(ballistics_check PRIVATE -Wall -Wextra -pedantic)

if(NOT CMAKE_CROSSCOMPILING)
    add_executable(ballistics_tests tests/ballistics_tests.cpp)
    target_link_libraries(ballistics_tests PRIVATE ballistics gtest gtest_main)
    include(GoogleTest)
    gtest_discover_tests(ballistics_tests)
endif()