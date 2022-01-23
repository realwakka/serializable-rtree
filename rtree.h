#ifndef RTREE_H_
#define RTREE_H_

#include <cairo.h>
#include <string>
#include <vector>
#include <iostream>

#include "box.h"

namespace rtree {

constexpr int dim = 2;
constexpr int fanout = 3;

template<int D, int F>
struct BasicNode {
  int size_;
  int rects_[F];
};

using Node = BasicNode<2, 3>;

template<int D>
struct BasicRect {
  BasicBox<D> box_;
  int child_;
  int id_;
};

using Rect = BasicRect<2>;

struct RTreeData {
  std::vector<Node> nodes_;
  std::vector<Rect> rects_;
  int root_rect_offset_;
};

class RTree {
 public:
  RTree();
  void insert(const Box& box, int id);
  void print();
  
  RTreeData data_;
};

class Writer {
 public:
  Writer();
  void write(const std::string& path, const RTree& rtree);
};

class Reader {
 public:
  Reader();
  RTree read(const std::string& path);
};

void print_as_image(const std::string& filename, const RTreeData& rtree);

}

#endif
