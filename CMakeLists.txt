project(serializable-rtree)

set (CMAKE_CXX_STANDARD 17)

include_directories(/usr/include/cairo)

add_executable(main main.cc rtree.cc)
target_link_libraries(main cairo)


