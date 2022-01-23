#include <iostream>
#include <vector>
#include <cassert>
#include <random>

#include "rtree.h"

// class Reader {
//   void read(const std::string& path);
//   std::vector<int> search(const Rect& query);
// };

// class Writer {
// public:
//   Writer();
//   void write(const std::string& path);
//   void insert(const Box& box, int id);
//   void print();

//   std::vector<Node> nodes_;
//   std::vector<Rect> rects_;
//   int root_rect_offset_;
// };

void insert_random_boxes(rtree::Writer& writer, size_t count) {
  std::random_device rd;
  std::mt19937 mt(rd());
  
  std::uniform_int_distribution<int> dist(0, 900);
  for(int i=0; i<8; ++i) {
    Box box;
    for(int j=0; j<2; ++j) {
      auto a = dist(mt);
      box.min_[j] = a;
      box.max_[j] = a + 100;
    }
    writer.insert(box, i);
    writer.print();
  }
}

int main() {

  std::cout << sizeof(Box) << std::endl;
  using namespace rtree;
  Writer writer;
  insert_random_boxes(writer, 8);
  print_as_image("test1.png", writer.rtree_);
  writer.write("test");
  return 0;
}