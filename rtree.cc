#include "rtree.h"

#include <cassert>
#include <limits>
#include <filesystem>
#include <fstream>
#include <queue>
#include "box.h"

namespace rtree {

template<int D, int F>
void refresh_rect(std::vector<BasicNode<F>>& nodes, std::vector<BasicRect<D>>& rects, int rect_offset) {
  auto& rect = rects[rect_offset];

  assert(rect.child_ >= 0);
  assert(rect.child_ < nodes.size());

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

std::pair<std::vector<int>, std::vector<int>> quadratic_split(const std::vector<int>& rect_offsets,
							      const std::vector<Rect>& rects) {

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

  auto result = quadratic_split(rect_offsets, rects);

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


int choose_subtree(const Node& node, const Rect& rect, const std::vector<Rect>& rects) {
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

void intersects_impl(const Box& box, const RTreeData& data, int rect_offset, std::vector<int>& result) {
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
  intersects_impl(box, data_, data_.root_rect_offset_, result);
  return result;
}

Writer::Writer() {}


void Writer::write(const std::string& path, const RTree& rtree) {
  std::filesystem::create_directory(path);

  const auto& data = rtree.data();
  auto node_path = path + "/nodes";
  {
    std::ofstream ofs;
    ofs.open(node_path.c_str(), std::ios::out | std::ios::binary);
    ofs << data.nodes_.size();
    ofs.write(reinterpret_cast<const char*>(data.nodes_.data()),
              sizeof(Node) * data.nodes_.size());
    ofs.close();
  }

  auto rect_path = path + "/rects";
  {
    std::ofstream ofs;
    ofs.open(rect_path.c_str(), std::ios::out | std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(&data.root_rect_offset_),
              sizeof(data.root_rect_offset_));
    ofs << data.rects_.size();
    ofs.write(reinterpret_cast<const char*>(data.rects_.data()),
              sizeof(Rect) * data.rects_.size());    
    ofs.close();
  }
}

void print_rect(const RTree& rtree, int offset) {
  std::cout << "|" << offset << "(" << rtree.rect(offset).child_ << ")"<< "|";
}

void print_node(const RTree& rtree, int offset) {
  std::cout << "#" << offset <<"{";
  const auto& node = rtree.node(offset);
  for(int i=0; i<node.size_; ++i)
    print_rect(rtree, node.rects_[i]);

  std::cout << "}";
}

void RTree::print() {
  std::vector<int> rects;
  rects.emplace_back(data_.root_rect_offset_);

  print_rect(*this, rects.front());
  std::cout << std::endl;
  while(!rects.empty()) {
    std::vector<int> next_rects;

    for(auto r : rects) {
      if (data_.rects_[r].child_ < 0)
	continue;

      print_node(*this, data_.rects_[r].child_);
      auto& node = data_.nodes_[data_.rects_[r].child_];
      for(auto i=0; i<node.size_; ++i) {
        next_rects.emplace_back(node.rects_[i]);
      }
    }

    std::cout << std::endl;
    rects = std::move(next_rects);
  }

  std::cout << std::endl;  
}


Reader::Reader() {}

RTree Reader::read(const std::string& path) {
  auto node_path = path + "/nodes";
  RTreeData data;  
  {
    std::ifstream ifs;
    ifs.open(node_path.c_str(), std::ifstream::binary);
    size_t size;
    ifs >> size;

    data.nodes_ = std::vector<Node>{size};
    ifs.read(reinterpret_cast<char*>(data.nodes_.data()),
             sizeof(Node) * size);
    ifs.close();
  }

  auto rect_path = path + "/rects";
  {
    std::ifstream ifs;
    ifs.open(rect_path.c_str(), std::ifstream::binary);
    ifs.read(reinterpret_cast<char*>(&data.root_rect_offset_),
             sizeof(data.root_rect_offset_));
    size_t size;
    ifs >> size;
    data.rects_ = std::vector<Rect>{size};
    ifs.read(reinterpret_cast<char*>(data.rects_.data()),
             sizeof(Rect) * size);
    ifs.close();
  }

  RTree rtree{data};
  return rtree;
}

std::vector<int> knn_impl(const RTreeData& data, const Point& query, int k) {
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
  return knn_impl(data_, query, k);
}


}
