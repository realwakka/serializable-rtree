project(serializable-rtree)

include_directories(/usr/include/cairo)

add_executable(main main.cc rtree.cc)
target_link_libraries(main cairo)
