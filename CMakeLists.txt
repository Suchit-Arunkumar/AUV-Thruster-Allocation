cmake_minimum_required(VERSION 3.16)
project(auv_thruster_allocation C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(alloc STATIC src/alloc.c)
target_include_directories(alloc PUBLIC include)
target_compile_options(alloc PRIVATE -Wall -Wextra)
if(UNIX)
  target_link_libraries(alloc PUBLIC m)
endif()
