# Configuration

cmake_minimum_required(VERSION 3.10)

project(MyGAL LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

# MyGAL

add_library(mygal INTERFACE)
target_include_directories(mygal INTERFACE include)

# Examples

add_subdirectory(examples)