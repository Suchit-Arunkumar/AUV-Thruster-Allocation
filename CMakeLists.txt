cmake_minimum_required(VERSION 3.16)
project(auv_thruster_allocation C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ALLOC_WERROR "Treat warnings as errors" ON)

set(ALLOC_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Wfloat-conversion)
if(ALLOC_WERROR)
  list(APPEND ALLOC_WARNINGS -Werror)
endif()

add_library(alloc STATIC src/alloc.c)
target_include_directories(alloc PUBLIC include)
# No fused multiply-add: keeps results bit-identical to the NumPy reference
# and to the Cortex-M4 bench build, which uses the same flag.
target_compile_options(alloc PRIVATE ${ALLOC_WARNINGS} -ffp-contract=off)
if(UNIX)
  target_link_libraries(alloc PUBLIC m)
endif()

enable_testing()

add_executable(test_alloc tests/c/test_alloc.c)
target_link_libraries(test_alloc PRIVATE alloc)
target_compile_options(test_alloc PRIVATE ${ALLOC_WARNINGS})
add_test(NAME c_unit COMMAND test_alloc)

add_executable(alloc_vectors tests/c/alloc_vectors.c)
target_link_libraries(alloc_vectors PRIVATE alloc)
target_compile_options(alloc_vectors PRIVATE ${ALLOC_WARNINGS})
