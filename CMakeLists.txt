cmake_minimum_required(VERSION 3.9)

project(hybrid_a_star LANGUAGES CXX)

include_directories("C:/Program Files (x86)/Eigen/eigen-3.4.0/")

set(CMAKE_CXXSTANDARD 17)

#set DEBUG
set(CMAKE_BUILD_TYPE DEBUG) 

aux_source_directory(. DIR_SRCS)
  
add_subdirectory(src)

add_executable(test main.cc)

aux_source_directory(. source_list)

target_link_libraries(test BasicFunciton)

set_target_properties(test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")