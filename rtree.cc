#include "rtree.h"

#include <cassert>
#include <limits>
#include <filesystem>
#include <fstream>
#include <queue>
#include "box.h"

namespace rtree {

template<int D, int F>
void refresh_rect(BasicNode<F>* nodes, BasicRect<D>* rects, int rect_offset) {
  auto& rect = rects[rect_offset];

  // assert(rect.child_ >= 0);
  // assert(rect.child_ < nodes.size());

  auto& child_node = nodes[rect.child_];

  for(int j=0; j<D; ++j) {  
    rect.box_.min_[j] = rects[child_node.rects_[0]].box_.min_[j];
    rect.box_.max_[j] = rects[child_node.rects_[0]].box_.max_[j];
  }

  for(int i=1; i<child_node.size_; ++i) {
    auto& child_rect = rects[child_node.rects_[i]];
    for(int j=0; j<D; ++j) {
      rect.box_.min_[j] = std::min(rect.box_.min_[j], child_rect.box_.min_[j]);
      rect.box_.max_[j] = std::max(rect.box_.max_[j], child_rect.box_.max_[j]);      
    }
  }
}

template<int D, int F>
void refresh_rect(std::vector<BasicNode<F>>& nodes, std::vector<BasicRect<D>>& rects, int rect_offset) {
  refresh_rect(nodes.data(), rects.data(), rect_offset);
}

template<int D>
std::pair<std::vector<int>, std::vector<int>> quadratic_split(const std::vector<int>& rect_offsets,
							      const BasicRect<D>* rects) {

  auto calc_deadspace = [](const auto& a, const auto& b) {
			  Box c = bounding_box(a,b);
			  return area(c) - area(a) - area(b);
			};

  int max_deadspace = std::numeric_limits<int>::lowest();

  int seed_a = -1;
  int seed_b = -1;
  
  // choose two seeds
  for(int i=0; i<rect_offsets.size(); ++i) {
    for(int j=1; j<rect_offsets.size(); ++j) {
      auto deadspace = calc_deadspace(rects[rect_offsets[i]].box_, rects[rect_offsets[j]].box_);
      if (max_deadspace < deadspace) {
	max_deadspace = deadspace;
	seed_a = rect_offsets[i];
	seed_b = rect_offsets[j];
      }
    }
  }
  std::vector<int> ret_a;
  ret_a.emplace_back(seed_a);
  
  std::vector<int> ret_b;
  ret_b.emplace_back(seed_b);

  Box group_a = rects[seed_a].box_;
  Box group_b = rects[seed_b].box_;

  // split
  for(auto offset : rect_offsets) {
    if (offset == seed_a || offset == seed_b)
      continue;

    auto enlargement_a = get_enlargement(group_a, rects[offset].box_);
    auto enlargement_b = get_enlargement(group_b, rects[offset].box_);

    if (enlargement_a > enlargement_b) {
      group_b = bounding_box(group_b, rects[offset].box_);
      ret_b.emplace_back(offset);
    } else {
      group_a = bounding_box(group_a, rects[offset].box_);      
      ret_a.emplace_back(offset);
    }
  }

  return std::make_pair(ret_a, ret_b);
}


int split(std::vector<Rect>& rects, std::vector<Node>& nodes, int fanout, int old_node_offset, int new_rect_offset) {
  Rect& new_rect = rects[new_rect_offset];
  Node new_node;

  auto& old_node = nodes[old_node_offset];
  std::vector<int> rect_offsets{old_node.rects_, old_node.rects_ + old_node.size_};
  rect_offsets.emplace_back(new_rect_offset);

  auto result = quadratic_split(rect_offsets, rects.data());

  for(int i=0; i<result.first.size(); ++i)
    old_node.rects_[i] = result.first[i];
  old_node.size_ = result.first.size();
  
  for(int i=0; i<result.second.size(); ++i)
    new_node.rects_[i] = result.second[i];
  new_node.size_ = result.second.size();

  int new_node_offset = nodes.size();
  nodes.emplace_back(new_node);

  Rect new_parent_rect;
  new_parent_rect.child_ = new_node_offset;
  new_parent_rect.id_ = -1;
  int new_parent_rect_offset = rects.size();
  rects.emplace_back(new_parent_rect);

  refresh_rect(nodes, rects, new_parent_rect_offset);

  return new_parent_rect_offset;
}


template<int D, int F>
int choose_subtree(const BasicNode<F>& node, const BasicRect<D>& rect, const BasicRect<D>* rects) {
  assert(node.size_ > 0);

  auto selected_rect_offset = node.rects_[0];
  auto min_enlargement = get_enlargement(rects[node.rects_[0]].box_, rect.box_);
  for(auto i=1; i<node.size_; ++i) {
    auto enlargement = get_enlargement(rects[node.rects_[i]].box_, rect.box_);
    if (min_enlargement > enlargement) {
      selected_rect_offset = node.rects_[i];
      min_enlargement = enlargement;
    }
  }
  return selected_rect_offset;
}

int choose_subtree(const Node& node, const Rect& rect, const std::vector<Rect>& rects) {
  return choose_subtree(node, rect, rects.data());
}


template<int D, int F>
int insert_recursive(int new_rect_offset, int target_node_offset,
                     std::vector<BasicNode<F>>& nodes, std::vector<BasicRect<D>>& rects) {
  auto& node = nodes[target_node_offset];
  const Rect& new_rect = rects[new_rect_offset];  

  if (node.size_ == 0 || rects[nodes[target_node_offset].rects_[0]].child_ == -1) { // leaf
    if (node.size_ == F) { // full need to split
      return split(rects, nodes, F, target_node_offset, new_rect_offset);
    } else {
      node.rects_[node.size_] = new_rect_offset;
      ++node.size_;
      return -1;
    }
  } else { // internal
    auto chosen_rect_offset = choose_subtree(node, new_rect, rects);
    auto chosen_node_offset = rects[chosen_rect_offset].child_;
    auto new_parent_rect_offset = insert_recursive(new_rect_offset, chosen_node_offset, nodes, rects);
    refresh_rect(nodes, rects, chosen_rect_offset);    
    if (new_parent_rect_offset == -1) {
      return -1;
    } else {
      if (node.size_ == F) { // full need to split
	return split(rects, nodes, F, target_node_offset, new_parent_rect_offset);
      } else {
	node.rects_[node.size_] = new_parent_rect_offset;
	++node.size_;
	return -1;
      }
    }
  }
}

template<int D, int F>
void init_rtree(BasicRTreeData<D, F>& rtree) {
  Node root_node;
  root_node.size_ = 0;
  for(int i=0; i<F; ++i)
    root_node.rects_[i] = -1;

  rtree.nodes_.emplace_back(root_node);

  Rect root_rect;
  root_rect.id_ = -1;
  root_rect.child_ = 0;

  rtree.rects_.emplace_back(root_rect);
  rtree.root_rect_offset_ = 0;
}

RTree::RTree() {
  init_rtree(data_);
}

RTree::RTree(const RTreeData& data) : data_{data} {}

void RTree::insert(const Box& box, int id) {
  Rect rect;
  rect.box_ = box;
  rect.child_ = -1;
  rect.id_ = id;
  
  auto offset = data_.rects_.size();
  data_.rects_.emplace_back(rect);

  auto root_node_offset = data_.rects_[data_.root_rect_offset_].child_;
  auto res = insert_recursive(offset, root_node_offset, data_.nodes_, data_.rects_);
  refresh_rect(data_.nodes_, data_.rects_, data_.root_rect_offset_);
  
  if (res >= 0) {
    Node new_root_node;
    new_root_node.rects_[0] = data_.root_rect_offset_;
    new_root_node.rects_[1] = res;
    new_root_node.size_ = 2;    

    root_node_offset = data_.nodes_.size();
    data_.nodes_.emplace_back(new_root_node);
    
    Rect root_rect;
    root_rect.child_ = root_node_offset;
    root_rect.id_ = -1;
    data_.root_rect_offset_ = data_.rects_.size();
    data_.rects_.emplace_back(root_rect);
    refresh_rect(data_.nodes_, data_.rects_, data_.root_rect_offset_);    
  }
}

template<int D, int F>
void intersects_impl(const Box& box, const StaticRTreeData<D, F>& data, int rect_offset, std::vector<int>& result) {
  const auto& rect = data.rects_[rect_offset];

  if (is_overlapped(rect.box_, box)) {
    if (rect.id_ >= 0)
      result.emplace_back(rect.id_);

    const auto& child_node = data.nodes_[rect.child_];

    for(int i=0; i<child_node.size_; ++i) {
      intersects_impl(box, data, child_node.rects_[i], result);
    }
  }
}

std::vector<int> RTree::intersects(const Box& box) {
  std::vector<int> result;
  StaticRTreeData<2, 3> static_data = {
    .nodes_ = data_.nodes_.data(),
    .rects_ = data_.rects_.data(),
    .root_rect_offset_ = data_.root_rect_offset_
  };
  intersects_impl(box, static_data, data_.root_rect_offset_, result);
  return result;
}


template<int D>
void print_rect(const BasicRect<D>* rects, int offset) {
  std::cout << "|" << offset << "(" << rects[offset].child_ << ")"<< "|";
}


template<int D, int F>
void print_node(const StaticRTreeData<D, F>& rtree, int offset) {
  std::cout << "#" << offset <<"{";
  const auto& node = rtree.nodes_[offset];
  for(int i=0; i<node.size_; ++i)
    print_rect(rtree.rects_, node.rects_[i]);

  std::cout << "}";
}

template<int D, int F>
void print_impl(const StaticRTreeData<D, F>& rtree) {
  std::vector<int> rects;
  rects.emplace_back(rtree.root_rect_offset_);

  print_rect(rtree.rects_, rects.front());
  std::cout << std::endl;
  while(!rects.empty()) {
    std::vector<int> next_rects;

    for(auto r : rects) {
      if (rtree.rects_[r].child_ < 0)
	continue;

      print_node(rtree, rtree.rects_[r].child_);
      auto& node = rtree.nodes_[rtree.rects_[r].child_];
      for(auto i=0; i<node.size_; ++i) {
        next_rects.emplace_back(node.rects_[i]);
      }
    }

    std::cout << std::endl;
    rects = std::move(next_rects);
  }

  std::cout << std::endl;  
}

void RTree::print() {
  print_impl(make_static(data()));
}

template<int D, int F>
std::vector<int> knn_impl(const StaticRTreeData<D, F>& data, const Point& query, int k) {
  auto cmp = [&data, &query] (int a, int b) {
               return get_mindist(data.rects_[a].box_, query) > get_mindist(data.rects_[b].box_, query);
             };

  std::priority_queue<int, std::vector<int>, decltype(cmp)> q{cmp};
  q.emplace(data.root_rect_offset_);

  std::vector<int> result;
  while(result.size() < k) {
    while(data.rects_[q.top()].child_ >= 0) {
      auto child_node_offset = data.rects_[q.top()].child_;     
      const auto& child_node = data.nodes_[child_node_offset];
      q.pop();

      for(int i=0; i < child_node.size_; ++i)
        q.emplace(child_node.rects_[i]);
    }

    result.emplace_back(q.top());
    q.pop();
  }

  return result;
}

std::vector<int> RTree::knn(const Point& query, int k) {
  return knn_impl<2, 3>(make_static(data_), query, k);
}

// return true if rect should be removed
bool remove_recursive(RTreeData& data, int rect_offset, int id) {
  auto& rect = data.rects_[rect_offset];
  if (rect.child_ >= 0) { // internal
    auto& child_node = data.nodes_[rect.child_];
    for(int i=0; i<child_node.size_; ++i) {
      auto res = remove_recursive(data, child_node.rects_[i], id);
      if (res) {
        auto removed_rect_offset = child_node.rects_[i];
        child_node.size_--;
        std::swap(child_node.rects_[child_node.size_], child_node.rects_[i]);
        return child_node.size_ == 0;
      }
    }
  } else { // leaf
    if (rect.id_ == id)
      return true;
    
  }
  return false;
}

void RTree::remove(int id) {
  remove_recursive(data_, data_.root_rect_offset_, id);
}


template<int D, int F, class MemoryProvider>
int Writer<D, F, MemoryProvider>::split(int fanout, int old_node_offset, int new_rect_offset) {
  Rect& new_rect = rects_[new_rect_offset];


  auto& old_node = nodes_[old_node_offset];
  std::vector<int> rect_offsets{old_node.rects_, old_node.rects_ + old_node.size_};
  rect_offsets.emplace_back(new_rect_offset);

  auto result = quadratic_split(rect_offsets, rects_);

  for(int i=0; i<result.first.size(); ++i)
    old_node.rects_[i] = result.first[i];
  old_node.size_ = result.first.size();

  // Node new_node;
  Node* new_node = insert_node();
  for(int i=0; i<result.second.size(); ++i)
    new_node->rects_[i] = result.second[i];
  new_node->size_ = result.second.size();

  int new_node_offset = header_->node_size_ - 1;
  // nodes_.emplace_back(new_node);

  // Rect new_parent_rect;
  Rect* new_parent_rect = insert_rect();  
  new_parent_rect->child_ = new_node_offset;
  new_parent_rect->id_ = -1;
  int new_parent_rect_offset = header_->rect_size_ - 1;
  // rects_.emplace_back(new_parent_rect);

  refresh_rect(nodes_, rects_, new_parent_rect_offset);

  return new_parent_rect_offset;
}


template<int D, int F, class MemoryProvider>
int Writer<D, F, MemoryProvider>::insert_recursive(int new_rect_offset, int target_node_offset) {
  auto& node = nodes_[target_node_offset];
  const Rect& new_rect = rects_[new_rect_offset];  

  if (node.size_ == 0 || rects_[nodes_[target_node_offset].rects_[0]].child_ == -1) { // leaf
    if (node.size_ == F) { // full need to split
      return split(F, target_node_offset, new_rect_offset);
    } else {
      node.rects_[node.size_] = new_rect_offset;
      ++node.size_;
      return -1;
    }
  } else { // internal
    auto chosen_rect_offset = choose_subtree(node, new_rect, rects_);
    auto chosen_node_offset = rects_[chosen_rect_offset].child_;
    auto new_parent_rect_offset = insert_recursive(new_rect_offset, chosen_node_offset);
    refresh_rect(nodes_, rects_, chosen_rect_offset);    
    if (new_parent_rect_offset == -1) {
      return -1;
    } else {
      if (node.size_ == F) { // full need to split
	return split(F, target_node_offset, new_parent_rect_offset);
      } else {
	node.rects_[node.size_] = new_parent_rect_offset;
	++node.size_;
	return -1;
      }
    }
  }
}


template<int D, int F, class MemoryProvider>
void Writer<D, F, MemoryProvider>::insert(const Box& box, int id) {
  Rect* rect = insert_rect();
  rect->box_ = box;
  rect->child_ = -1;
  rect->id_ = id;
    
  auto offset = header_->rect_size_ - 1;
  
  auto root_node_offset = rects_[header_->root_rect_offset_].child_;
  auto res = insert_recursive(offset, root_node_offset);
  refresh_rect(nodes_, rects_, header_->root_rect_offset_);    

  if (res >= 0) {
    auto new_root_node = insert_node();
    new_root_node->rects_[0] = header_->root_rect_offset_;
    new_root_node->rects_[1] = res;
    new_root_node->size_ = 2;

    root_node_offset = header_->node_size_ - 1;
      
    auto root_rect = insert_rect();
    root_rect->child_ = root_node_offset;
    root_rect->id_ = -1;
    header_->root_rect_offset_ = header_->rect_size_ - 1;
    refresh_rect(nodes_, rects_, header_->root_rect_offset_);
  }
}

template<int D, int F, class MemoryProvider>
void Writer<D, F, MemoryProvider>::print_tree() {
  print_impl(static_data());
}

template<int D, int F, class MemoryProvider>
std::vector<int> Reader<D, F, MemoryProvider>::intersects(const Box& box) {
  std::vector<int> result;    
  intersects_impl(box, data_, data_.root_rect_offset_, result);
  return result;
}

template<int D, int F, class MemoryProvider>
std::vector<int> Reader<D, F, MemoryProvider>::knn(const BasicPoint<D>& query, int k) {
  return knn_impl(data_, query, k);  
}


template class Reader<2, 3, MappedFileProvider>;
template class Writer<2, 3, MappedFileProvider>;



}
