cmake_minimum_required(VERSION 3.20)
project(pathfinder_platformer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_EXAMPLES "Build examples" ON)

add_library(pathfinder
        src/PathFinder.cpp
)

target_include_directories(pathfinder
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (BUILD_EXAMPLES)
    add_executable(pathfinder_cli examples/cli_main.cpp)
    target_link_libraries(pathfinder_cli PRIVATE pathfinder)
endif()
