# Configuration

cmake_minimum_required(VERSION 3.8.2)

project(MyGAL LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MyGAL

add_library(mygal INTERFACE)
target_include_directories(mygal INTERFACE include)

# Code coverage

option(CODE_COVERAGE "Enable coverage reporting" OFF)
if(CODE_COVERAGE)
    target_compile_options(mygal INTERFACE
        -O0
        -g
        --coverage)
    target_link_libraries(mygal INTERFACE --coverage)
endif(CODE_COVERAGE)

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