#include <iostream>
#include <vector>
#include <cassert>
#include <random>

#include "rtree.h"
#include "image_util.h"

rtree::Box create_random_box() {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_int_distribution<int> dist(0, 900);
  
  rtree::Box box;  
  for(int j=0; j<2; ++j) {
    auto a = dist(mt);
    box.min_[j] = a;
    box.max_[j] = a + 100;
  }

  return box;
}

std::vector<rtree::Box> insert_random_boxes(rtree::RTree& rtree, size_t count) {

  std::vector<rtree::Box> boxes;
  for(int i=0; i<count; ++i) {
    auto box = create_random_box();
    rtree.insert(box, i);
    boxes.emplace_back(box);
  }
  return boxes;
}

void print_box(const rtree::Box& box) {
  std::cout << "(";
  for(int j=0; j<2; ++j) {
    std::cout << box.min_[j] << "," << box.max_[j] << "|";
  }
  std::cout << ")" << std::endl;
}

int main() {
  using namespace rtree;

  RTree rtree{};
  auto boxes = insert_random_boxes(rtree, 16);

  Point p;
  p.value_[0] = 500;
  p.value_[1] = 500;

  auto knn_result = rtree.knn(p, 5);

  std::cout << "KNN results: ";
  for(auto&& i : knn_result) {
    std::cout << i << " ";
  }
  std::cout << std::endl;

  util::print_as_image_with_query_point("output.png", rtree.data(), p);
  return 0;
}
