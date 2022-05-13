# Configuration

cmake_minimum_required(VERSION 3.8.2)

project(MyGAL LANGUAGES CXX VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MyGAL

add_library(mygal INTERFACE)
target_include_directories(mygal INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>)

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

# Install

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/MyGALConfigVersion.cmake
    VERSION 
    COMPATIBILITY AnyNewerVersion)
configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/MyGALConfig.cmake.in
    ${CMAKE_BINARY_DIR}/MyGALConfig.cmake
    INSTALL_DESTINATION lib/cmake/MyGAL)

install(TARGETS mygal EXPORT MyGALTargets
    INCLUDES DESTINATION include)
install(EXPORT MyGALTargets
    FILE MyGALTargets.cmake
    DESTINATION lib/cmake/MyGAL)
install(FILES 
    ${CMAKE_BINARY_DIR}/MyGALConfigVersion.cmake
    ${CMAKE_BINARY_DIR}/MyGALConfig.cmake
    DESTINATION lib/cmake/MyGAL)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

