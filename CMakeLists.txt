# Configuration

cmake_minimum_required(VERSION 3.8.2)

project(MyGAL LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

# MyGAL

add_library(mygal INTERFACE)
target_include_directories(mygal INTERFACE include)

# Examples

option (BUILD_EXAMPLES "Build the examples." ON)
if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests

option (BUILD_TESTING "Build the testing tree." ON)
if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()