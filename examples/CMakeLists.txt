# Executable

add_executable(example main.cpp)
target_link_libraries(example PRIVATE mygal)
target_compile_options(example PRIVATE 
    -Wall 
    -Wextra 
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wpedantic
    -Wmisleading-indentation
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wuseless-cast
    -Wdouble-promotion)
target_compile_features(example PRIVATE cxx_std_14)

# SFML 

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})
find_package(SFML 2.4 REQUIRED graphics window system)
if(SFML_FOUND)
    target_include_directories(example PRIVATE ${SFML_INCLUDE_DIR})
    target_link_libraries(example PRIVATE ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
else()
    message(FATAL_ERROR "SFML not found")
endif()