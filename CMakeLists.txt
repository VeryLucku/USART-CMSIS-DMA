cmake_minimum_required(VERSION 3.20)

project("TT" C CXX ASM)

include(cmake/st-project.cmake)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})

set(CMAKE_EXE_LINKER_FLAGS  "-u _printf_float")

#target_compile_options(${PROJECT_NAME} PUBLIC -O3 -fno-exceptions)