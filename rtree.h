#ifndef RTREE_H_
#define RTREE_H_

#include <string>
#include <vector>
#include <iostream>

#include "box.h"

namespace rtree {

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
  explicit RTree(const RTreeData& data);
  void insert(const Box& box, int id);
  std::vector<int> intersects(const Box& box);
  std::vector<int> knn(const Point& p, int k);
  void print();

  const RTreeData& data() const { return data_; }
  const Rect& root() const { return data_.rects_[data_.root_rect_offset_]; }
  const Rect& rect(int offset) const { return data_.rects_[offset]; }
  const Node& node(int offset) const { return data_.nodes_[offset]; }

 private:
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

}

#endif
