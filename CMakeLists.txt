cmake_minimum_required(VERSION 3.20)

# БЛОК ТЕСТУВАННЯ
# Увмкнення тестів
enable_testing()
if(NOT CMAKE_CROSSCOMPILING)

    # Завантаження GTest
    include(FetchContent)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        v1.15.2
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

endif()

# Загальний CMake-проєкт для матеріалів блоку 2.
# Кожна homework_* директорія підключається окремо.
project(section2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Домашні роботи. По мірі появи додаємо кожну окремим add_subdirectory.
add_subdirectory(homework_04)
add_subdirectory(homework_06)
