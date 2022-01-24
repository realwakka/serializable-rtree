cmake_minimum_required(VERSION 3.16)

project(serializable-rtree)

set (CMAKE_CXX_STANDARD 17)

include_directories(/usr/include/cairo)

add_library(rtree
  rtree.cc
  )

add_executable(test test.cc)
target_link_libraries(test rtree cairo)


