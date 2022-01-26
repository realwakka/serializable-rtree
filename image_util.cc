#include "image_util.h"

#include <cairo.h>

namespace rtree {
namespace util {

void print_tree(cairo_t* cr, const RTreeData& rtree) {
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
}

void print_query(cairo_t* cr, const Box& query) {
  cairo_set_source_rgba(cr, 255, 0, 0, 255);
  cairo_set_line_width(cr, 2);

  cairo_rectangle (cr, query.min_[0], query.min_[1],
                   query.max_[0] - query.min_[0], query.max_[1] - query.min_[1]);
  cairo_stroke (cr);

  cairo_save(cr);
  cairo_set_font_size(cr, 30);
  cairo_move_to(cr, query.min_[0], query.min_[1]);
  cairo_show_text(cr, "Q");
  cairo_restore(cr);
}

void print_as_image(const std::string& filename, const RTreeData& rtree) {
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

  print_tree(cr, rtree);
  status = cairo_surface_write_to_png(surface, filename.c_str());

  if (status != CAIRO_STATUS_SUCCESS) {
    std::cout << cairo_status_to_string(status) << std::endl;
  }

  cairo_destroy(cr);  
}

void print_as_image_with_query(const std::string& filename, const RTreeData& rtree, const Box& query) {
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

  print_tree(cr, rtree);
  print_query(cr, query);

  status = cairo_surface_write_to_png(surface, filename.c_str());

  if (status != CAIRO_STATUS_SUCCESS) {
    std::cout << cairo_status_to_string(status) << std::endl;
  }

  cairo_destroy(cr);  
}

}
}
