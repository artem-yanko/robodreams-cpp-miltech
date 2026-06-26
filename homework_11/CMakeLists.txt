add_library(hw11_drone_link_adapter src/io/drone_link_adapter.cpp)
target_include_directories(hw11_drone_link_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw11_drone_link_adapter PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_gpio_controller src/io/gpio_controller.cpp)
target_include_directories(hw11_gpio_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw11_gpio_controller PRIVATE -Wall -Wextra -pedantic)

add_executable(homework_11_app src/main.cpp)
target_include_directories(homework_11_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(homework_11_app PRIVATE hw11_drone_link_adapter hw11_gpio_controller)
target_compile_options(homework_11_app PRIVATE -Wall -Wextra -pedantic)
