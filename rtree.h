#ifndef RTREE_H_
#define RTREE_H_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <string.h>

#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>

#include <cassert>

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
  
  void remove(int id);
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

struct Header {
  char magic_[6];
  int version_;
  int dimension_;
  int fanout_;
  int node_size_;
  int rect_size_;
  int root_rect_offset_;
};

class NewWriter {
 public:
  void write(const std::string& path, const RTreeData& data) {
    std::ofstream ofs;
    ofs.open(path.c_str(), std::ios::out | std::ios::binary);

    Header header;
    strncpy(header.magic_, "sidong", sizeof(header.magic_));
    header.version_ = 1;
    header.dimension_ = 2;
    header.fanout_ = 3;
    header.node_size_ = data.nodes_.size();
    header.rect_size_ = data.rects_.size();
    header.root_rect_offset_ = data.root_rect_offset_;

    ofs.write(reinterpret_cast<const char*>(&header), sizeof(header));
    ofs.write(reinterpret_cast<const char*>(data.nodes_.data()),
              data.nodes_.size() * sizeof(decltype(data.nodes_)::value_type));
    ofs.write(reinterpret_cast<const char*>(data.rects_.data()),
              data.rects_.size() * sizeof(decltype(data.rects_)::value_type));
  }
};

template<int D, int F>
struct StaticRTreeData {
  const BasicNode<F>* nodes_;
  const BasicRect<D>* rects_;
  int root_rect_offset_;
};

template<int D, int F>
class NewReader {
 public:
  NewReader(const std::string& path) : fd_{-1}, data_{nullptr, nullptr, 0} {

    fd_ = open(path.c_str(), O_RDONLY);

    struct stat s;
    auto status = fstat(fd_, &s);
    auto size = s.st_size;

    const char* mapped = reinterpret_cast<const char*>(mmap(0, size, PROT_READ, MAP_PRIVATE, fd_, 0));
    const Header* header = reinterpret_cast<const Header*>(mapped);
    assert(!strncmp(header->magic_, "sidong", 6));
    assert(header->version_ == 1);
    assert(header->dimension_ == D);
    assert(header->fanout_ == F);

    const BasicNode<F>* nodes = reinterpret_cast<const BasicNode<F>*>(&mapped[sizeof(Header)]);
    const BasicRect<D>* rects = reinterpret_cast<const BasicRect<D>*>(&nodes[header->node_size_]);

    data_.nodes_ = nodes;
    data_.rects_ = rects;
    data_.root_rect_offset_ = header->root_rect_offset_;
    // = {
    //   .nodes_ = nodes,
    //   .rects_ = rects,
    //   .root_rect_offset_ = header->root_rect_offset_
    // };
  }

  
  std::vector<int> intersects(const Box& box);
  std::vector<int> knn(const BasicPoint<D>& p, int k);

 private:
  StaticRTreeData<D, F> data_;
  int fd_;
};



}

#endif
