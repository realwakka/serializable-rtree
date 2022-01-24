#include <iostream>
#include <vector>
#include <cassert>
#include <random>

#include "rtree.h"

void insert_random_boxes(rtree::RTree& rtree, size_t count) {
  std::random_device rd;
  std::mt19937 mt(rd());
  
  std::uniform_int_distribution<int> dist(0, 900);
  for(int i=0; i<count; ++i) {
    rtree::Box box;
    for(int j=0; j<2; ++j) {
      auto a = dist(mt);
      box.min_[j] = a;
      box.max_[j] = a + 100;
    }
    rtree.insert(box, i);
    rtree.print();
  }
}

int main() {
  using namespace rtree;

  RTree rtree{};
  insert_random_boxes(rtree, 8);
  print_as_image("test1.png", rtree.data_);
  Writer writer{};
  writer.write("test", rtree);

  Reader reader{};
  auto loaded = reader.read("test");

  loaded.print();
  return 0;
}
