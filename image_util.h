#ifndef RTREE_IMAGE_UTIL_H_
#define RTREE_IMAGE_UTIL_H_

#include <string>

#include "rtree.h"

namespace rtree {
namespace util {

void print_as_image(const std::string& filename, const StaticRTreeData<2, 3>& rtree);
void print_as_image_with_query(const std::string& filename, const StaticRTreeData<2, 3>& rtree, const Box& query);
void print_as_image_with_query_point(const std::string& filename, const StaticRTreeData<2, 3>& rtree, const Point& query);

}
}

#endif
