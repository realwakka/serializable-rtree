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
#include <unistd.h>

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
const StaticRTreeData<D, F> make_static(const BasicRTreeData<D, F>& data) {
  return StaticRTreeData<D, F>{
    data.nodes_.data(),
    data.rects_.data(),
    data.root_rect_offset_
  };
}

class MappedFileProvider {
 public:
  MappedFileProvider(const std::string& path) {
    fd_ = open(path.c_str(), O_RDWR | O_CREAT, 0666);
    if (fd_ < 0) {
      perror(path.c_str());
    }

    struct stat s;
    auto status = fstat(fd_, &s);
    mapped_size_ = s.st_size;

    mapped_ = reinterpret_cast<char*>(mmap(0, mapped_size_, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd_, 0));
    if (mapped_ == MAP_FAILED) {
      perror("failed MappedFileProvider mmap");
      exit(1);
    }
  }

  ~MappedFileProvider() {
    if (mapped_)
      munmap(mapped_, mapped_size_);
    if (fd_ > 0)
      close(fd_);
  }

  char* addr() { return mapped_; }
  size_t size() const { return mapped_size_; }

  char* increase(size_t size) {
    char* temp = new char[mapped_size_];
    if (!temp) {
      perror("failed increase new temp");
      exit(1);
    }
      
    memcpy(temp, mapped_, mapped_size_);
    if (munmap(mapped_, mapped_size_) < 0) {
      perror("failed increase munmap");
      exit(1);
    }

    
    if (ftruncate(fd_, mapped_size_ + size) < 0) {
      perror("failed increase ftruncate");
      exit(1);
    }

    mapped_ = reinterpret_cast<char*>(mmap(0, mapped_size_ + size, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd_, 0));
    if (mapped_ == MAP_FAILED) {
      perror("failed increase mmap");
      exit(1);
    }

    memcpy(mapped_, temp, mapped_size_);
    delete[] temp;
    mapped_size_ += size;
    return mapped_;
  }
 private:
  int fd_;
  char* mapped_;
  size_t mapped_size_;
};

template<int D, int F, class MemoryProvider>
class NewReader {
 public:

  template<typename... Args>
  NewReader(Args&&... args) : provider_{std::forward<Args>(args)...}, data_{nullptr, nullptr, 0} {
    auto addr = provider_.addr();
    const Header* header = reinterpret_cast<const Header*>(addr);
    assert(!strncmp(header->magic_, "sidong", 6));
    assert(header->version_ == 1);
    assert(header->dimension_ == D);
    assert(header->fanout_ == F);

    const BasicNode<F>* nodes = reinterpret_cast<const BasicNode<F>*>(&addr[sizeof(Header)]);
    const BasicRect<D>* rects = reinterpret_cast<const BasicRect<D>*>(&nodes[header->node_size_]);

    data_.nodes_ = nodes;
    data_.rects_ = rects;
    data_.root_rect_offset_ = header->root_rect_offset_;
  }
  
  
  std::vector<int> intersects(const Box& box);
  std::vector<int> knn(const BasicPoint<D>& p, int k);

 private:
  MemoryProvider provider_;
  StaticRTreeData<D, F> data_;
};

template<int D, int F, class MemoryProvider>
class Writer2 {
 public:
  BasicNode<F>* insert_node() {
    header_ = reinterpret_cast<Header*>(provider_.increase(sizeof(BasicRect<D>)));
    nodes_ = reinterpret_cast<BasicNode<F>*>(header_ + 1);
    rects_ = reinterpret_cast<BasicRect<D>*>(&nodes_[header_->node_size_]);
    auto new_node = &nodes_[header_->node_size_];
    header_->node_size_++;        
    memmove(&nodes_[header_->node_size_], rects_, sizeof(BasicRect<D>) * header_->rect_size_);

    rects_ = reinterpret_cast<BasicRect<D>*>(&nodes_[header_->node_size_]);
    return new_node;
  }

  BasicRect<D>* insert_rect() {
    header_ = reinterpret_cast<Header*>(provider_.increase(sizeof(BasicRect<D>)));
    nodes_ = reinterpret_cast<BasicNode<F>*>(&header_[1]);
    rects_ = reinterpret_cast<BasicRect<D>*>(&nodes_[header_->node_size_]);    
    auto new_rect = &rects_[header_->rect_size_];
    header_->rect_size_++;
    return new_rect;
  }

  template<typename... Args>    
  Writer2(Args&&... args) : provider_{std::forward<Args>(args)...} {
    provider_.increase(sizeof(Header));

    header_ = reinterpret_cast<Header*>(provider_.addr());
    strncpy(header_->magic_, "sidong", sizeof(header_->magic_));
    header_->version_ = 1;
    header_->dimension_ = 2;
    header_->fanout_ = 3;
    header_->node_size_ = 0;
    header_->rect_size_ = 0;
    header_->root_rect_offset_ = 0;

    auto root_node = insert_node();
    root_node->size_ = 0;
    for(int i=0; i<F; ++i)
      root_node->rects_[i] = -1;

    auto root_rect = insert_rect();    
    root_rect->id_ = -1;
    root_rect->child_ = 0;
  }

  void insert(const Box& box, int id);

  StaticRTreeData<D, F> static_data() {
    return {nodes_, rects_, header_->root_rect_offset_};
  }

  void print_tree();

 private:
  int insert_recursive(int new_rect_offset, int target_node_offset);
  int split(int fanout, int old_node_offset, int new_rect_offset);

 private:
  MemoryProvider provider_;
  Header* header_;
  BasicRect<D>* rects_;
  BasicNode<F>* nodes_;  
};

}

#endif
