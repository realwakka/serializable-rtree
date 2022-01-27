cmake_minimum_required(VERSION 3.16)

project(serializable-rtree)

set (CMAKE_CXX_STANDARD 17)



add_library(rtree
  rtree.cc
  )

add_library(rtree_img
  image_util.cc
  )

target_include_directories(rtree_img PUBLIC /usr/include/cairo)

add_executable(test test.cc)
target_link_libraries(test rtree rtree_img cairo)

add_executable(test_knn test_knn.cc)
target_link_libraries(test_knn rtree rtree_img cairo)




