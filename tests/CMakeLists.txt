# Executables

add_executable(line line.cpp)
target_link_libraries(line PRIVATE mygal)

add_executable(grid grid.cpp)
target_link_libraries(grid PRIVATE mygal)

add_executable(circle circle.cpp)
target_link_libraries(circle PRIVATE mygal)

add_executable(random random.cpp)
target_link_libraries(random PRIVATE mygal)

# Tests

foreach(n RANGE 2 100)
    add_test(NAME "line${n}" COMMAND line ${n})
endforeach()

foreach(n RANGE 2 20)
    add_test(NAME "grid${n}" COMMAND grid ${n} ${n})
endforeach()

foreach(n RANGE 2 100)
    add_test(NAME "circle${n}" COMMAND circle ${n})
endforeach()

foreach(n RANGE 2 100)
    add_test(NAME "random${n}" COMMAND random ${n})
endforeach()