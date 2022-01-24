#ifndef BOX_H_
#define BOX_H_

namespace rtree {

template <int D>
struct BasicBox {
  int min_[D];
  int max_[D];
  static const int dim_ = D;
};

using Box = BasicBox<2>;

template <int D>
int area(const BasicBox<D>& box) {
  int a = 1;
  for(auto i=0; i<D; ++i)
    a *= box.max_[i] - box.min_[i];
  return a;
}

template <int D>
bool is_overlapped(const BasicBox<D>& r1, const BasicBox<D>& r2) {
  for(auto i=0; i<D; ++i)
    if (r1.min_[i] > r2.max_[i] || r1.max_[i] < r2.min_[i])
      return false;
  return true;
}

template <int D>
int get_enlargement(const BasicBox<D>& old_box, const BasicBox<D>& new_box) {
  BasicBox<D> sum_rect;

  for(auto i=0; i<D; ++i) {
    sum_rect.min_[i] = std::min(old_box.min_[i], new_box.min_[i]);
    sum_rect.max_[i] = std::max(old_box.max_[i], new_box.max_[i]);
  }
  
  return area(sum_rect) - area(old_box);
}

template <int D>
BasicBox<D> bounding_box(const BasicBox<D>& a, const BasicBox<D>& b) {
  BasicBox<D> ret;
  for(auto i=0; i<D; ++i) {
    ret.min_[i] = std::min(a.min_[i], b.min_[i]);
    ret.max_[i] = std::max(a.max_[i], b.max_[i]);
  }
  return ret;
}

}

#endif
