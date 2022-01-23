#include "rtree.h"

#include <cassert>
#include <limits>
#include <filesystem>
#include <fstream>

namespace rtree {

void refresh_rect(std::vector<Node>& nodes, std::vector<Rect>& rects, int rect_offset) {
  auto& rect = rects[rect_offset];

  assert(rect.child_ >= 0);
  assert(rect.child_ < nodes.size());

  auto& child_node = nodes[rect.child_];

  for(int j=0; j<dim; ++j) {  
    rect.box_.min_[j] = rects[child_node.rects_[0]].box_.min_[j];
    rect.box_.max_[j] = rects[child_node.rects_[0]].box_.max_[j];
  }

  for(int i=1; i<child_node.size_; ++i) {
    auto& child_rect = rects[child_node.rects_[i]];
    for(int j=0; j<dim; ++j) {
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

  // std::cout << "seed a : " << seed_a << std::endl;
  // std::cout << "seed b : " << seed_b << std::endl;

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

int insert_recursive(int new_rect_offset, int target_node_offset, std::vector<Node>& nodes, std::vector<Rect>& rects) {
  auto& node = nodes[target_node_offset];
  const Rect& new_rect = rects[new_rect_offset];  

  if (node.size_ == 0 || rects[nodes[target_node_offset].rects_[0]].child_ == -1) { // leaf
    if (node.size_ == fanout) { // full need to split
      return split(rects, nodes, fanout, target_node_offset, new_rect_offset);
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
      if (node.size_ == fanout) { // full need to split
	return split(rects, nodes, fanout, target_node_offset, new_parent_rect_offset);
      } else {
	node.rects_[node.size_] = new_parent_rect_offset;
	++node.size_;
	return -1;
      }
    }
  }
}

void init_rtree(RTree& rtree) {
  Node root_node;
  root_node.size_ = 0;
  for(int i=0; i<fanout; ++i)
    root_node.rects_[i] = -1;

  rtree.nodes_.emplace_back(root_node);

  Rect root_rect;
  root_rect.child_ = 0;

  rtree.rects_.emplace_back(root_rect);
  rtree.root_rect_offset_ = 0;
}

Writer::Writer() {
  init_rtree(rtree_);
}


void Writer::write(const std::string& path) {
  // std::filesystem::path fs_path{path};

  
  std::filesystem::create_directory(path);
  const RTree& rtree = rtree_;
  auto node_path = path + "/nodes";
  {
    std::ofstream ofs;
    ofs.open(node_path.c_str(), std::ios::out | std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(rtree.nodes_.data())
              , sizeof(Node) * rtree.nodes_.size());
    ofs.close();
  }

  auto rect_path = path + "/rects";
  {
    std::ofstream ofs;
    ofs.open(rect_path.c_str(), std::ios::out | std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(&rtree.root_rect_offset_),
              sizeof(rtree.root_rect_offset_));    
    ofs.write(reinterpret_cast<const char*>(rtree.rects_.data()),
              sizeof(Rect) * rtree.rects_.size());    
    ofs.close();
  }
}

void Writer::insert(const Box& box, int id) {
  Rect rect;
  rect.box_ = box;
  rect.child_ = -1;
  rect.id_ = id;
  
  auto offset = rtree_.rects_.size();
  rtree_.rects_.emplace_back(rect);

  auto root_node_offset = rtree_.rects_[rtree_.root_rect_offset_].child_;
  auto res = insert_recursive(offset, root_node_offset, rtree_.nodes_, rtree_.rects_);
  refresh_rect(rtree_.nodes_, rtree_.rects_, rtree_.root_rect_offset_);
  
  if (res >= 0) {
    Node new_root_node;
    new_root_node.rects_[0] = rtree_.root_rect_offset_;
    new_root_node.rects_[1] = res;
    new_root_node.size_ = 2;    

    root_node_offset = rtree_.nodes_.size();
    rtree_.nodes_.emplace_back(new_root_node);
    
    Rect root_rect;
    root_rect.child_ = root_node_offset;
    rtree_.root_rect_offset_ = rtree_.rects_.size();
    rtree_.rects_.emplace_back(root_rect);
    refresh_rect(rtree_.nodes_, rtree_.rects_, rtree_.root_rect_offset_);    
  }
}

void Writer::print() {
  std::vector<int> rects;
  rects.emplace_back(rtree_.root_rect_offset_);
  std::cout << "|" << rtree_.root_rect_offset_ << "|" << std::endl;
  while(!rects.empty()) {
    std::vector<int> next_rects;

    for(auto r : rects) {
      if (rtree_.rects_[r].child_ < 0)
	continue;
      auto& node = rtree_.nodes_[rtree_.rects_[r].child_];
      std::cout << "(" << rtree_.rects_[r].child_ << ")";
      std::cout << "|";
      for(auto i=0; i<node.size_; ++i) {
	std::cout << node.rects_[i] << " ";
	next_rects.emplace_back(node.rects_[i]);
      }
      std::cout << "|";      
    }

    std::cout << std::endl;
    rects = std::move(next_rects);
  }
}

void print_as_image(const std::string& filename, const RTree& rtree) {
  cairo_surface_t *surface;
  cairo_t *cr;
  cairo_status_t status;
  surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 1000, 1000);
  status = cairo_surface_status(surface);
  if (status != CAIRO_STATUS_SUCCESS) {
    std::cout << cairo_status_to_string(status) << std::endl;
    return;
  }
  
  cr = cairo_create(surface);

  cairo_set_source_rgb(cr, 255, 255, 255);
  cairo_paint(cr);

  auto set_color_by_level = [cr](auto level) {
			      switch(level%3) {
			      case 0:
				cairo_set_source_rgba(cr, 0, 0, 0, 255);
				break;
			      case 1:
				cairo_set_source_rgba(cr, 255, 0, 0, 255);
				break;
			      case 2:
				cairo_set_source_rgba(cr, 0, 255, 0, 255);
				break;
			      }
			    };

  auto level = 0;

  set_color_by_level(level);
  cairo_set_line_width(cr, 1);

  std::vector<int> tmp_rects;
  tmp_rects.emplace_back(rtree.root_rect_offset_);
  while(!tmp_rects.empty()) {
    std::vector<int> next_rects;

    for(auto r : tmp_rects) {
      auto& box = rtree.rects_[r].box_;
      cairo_rectangle (cr, box.min_[0], box.min_[1],
		       box.max_[0] - box.min_[0], box.max_[1] - box.min_[1]);
      cairo_stroke (cr);

      cairo_save(cr);
      cairo_set_font_size(cr, 30);
      cairo_move_to(cr, box.min_[0], box.min_[1]);

      std::string rect_no = std::to_string(r);
      cairo_show_text(cr, rect_no.c_str());
      cairo_restore(cr);
      
      if (rtree.rects_[r].child_ < 0)
	continue;
      auto& node = rtree.nodes_[rtree.rects_[r].child_];
      for(auto i=0; i<node.size_; ++i) {
	next_rects.emplace_back(node.rects_[i]);
      }
    }
    tmp_rects = std::move(next_rects);
    ++level;
    set_color_by_level(level);
  }
 

  status = cairo_surface_write_to_png(surface, filename.c_str());
  if (status != CAIRO_STATUS_SUCCESS) {
    std::cout << cairo_status_to_string(status) << std::endl;
  }
}


}