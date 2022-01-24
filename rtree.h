#ifndef RTREE_H_
#define RTREE_H_

#include <cairo.h>
#include <string>
#include <vector>
#include <iostream>

#include "box.h"

namespace rtree {

template <int D>
struct Dimension {
  static const int value = D;
};

template<int F>
struct BasicNode {
  int size_;
  int rects_[F];
};

using Node = BasicNode<3>;

template<int D>
struct BasicRect {
  BasicBox<D> box_;
  int child_;
  int id_;
};

using Rect = BasicRect<2>;

template<int D, int F>
struct BasicRTreeData {
  std::vector<BasicNode<F>> nodes_;
  std::vector<BasicRect<D>> rects_;
  int root_rect_offset_;
};

using RTreeData = BasicRTreeData<2, 3>;

class RTree {
 public:
  RTree();
  void insert(const Box& box, int id);
  std::vector<int> intersects(const Box& box);
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
void print_as_image_with_query(const std::string& filename, const RTreeData& rtree, const Box& query);

}

#endif
