#include "kdtree/kdtree.hpp"

namespace kdtree {

/**
 * @brief pretty print a point.
 * @code {.cpp}
 * std::cout << point << '\n';
 * @endcode
 */
template <typename coordinate_t, typename value_t, size_t dimensions>
std::ostream& operator<<(std::ostream& ostream, const point<coordinate_t, dimensions>& pt) {
    static_assert(std::is_floating_point_v<coordinate_t> || std::is_integral_v<coordinate_t>,
                  "coordinate_t must be an integer of floating point");
    ostream << '(';
    ostream << pt.get_element_in_dimension(0) << ", ";
    for (std::size_t i = 1; i < dimensions; ++i) {
        ostream << pt.get_element_in_dimension(i);
    }
    ostream << ')';
    return ostream;
}

/**
 * Finds the nearest point in the tree to the given point.
 * It is not valid to call this function if the tree is empty.
 *
 * @param pt a point
 * @return the nearest point in the tree to the given point
 */
template <typename coordinate_t, typename value_t, std::size_t dimensions>
auto kdtree<coordinate_t, value_t, dimensions>::nearest(const point_t& pt)
    -> std::optional<std::pair<point_t, value_t>> {
    if (root_ == nullptr) {
        return std::nullopt;
    }
    best_ = nullptr;
    n_visited_ = 0;
    best_distance_ = 0;
    nearest_(root_, pt, 0);
    return std::make_pair(best_->point_, best_->value_);
}

template <typename coordinate_t, typename value_t, size_t dimensions>
auto kdtree<coordinate_t, value_t, dimensions>::make_tree_(std::size_t begin, std::size_t end,
                                                           std::size_t index) -> node_t* {
    if (end <= begin) {
        return nullptr;
    }
    std::size_t n = begin + (end - begin) / 2;
    auto itr = nodes_.begin();
    // compute median
    std::nth_element(itr + begin, itr + n, itr + end, dimension_comparator(index));
    index = (index + 1) % dimensions;
    nodes_[n].left_ = make_tree_(begin, n, index);
    nodes_[n].right_ = make_tree_(n + 1, end, index);
    return &nodes_[n];
}

template <typename coordinate_t, typename value_t, std::size_t dimensions>
auto kdtree<coordinate_t, value_t, dimensions>::nearest_(node_t* root, const point_t& point,
                                                         std::size_t index) -> void {
    if (root == nullptr) {
        return;
    }
    ++n_visited_;
    const auto distance = root->distance(point);
    // update best candidate if distance is better than the current best.
    if (best_ == nullptr || distance < best_distance_) {
        best_distance_ = distance;
        best_ = root;
    }
    // can not get a better distance
    if (best_distance_ == 0) {
        return;
    }
    // if dx is positive then point is less than root in this dimension,
    // and we should go left. otherwise right.
    const auto dx = root->get_element_in_dimension(index) - point.get_element_in_dimension(index);
    auto less_than = dx > 0;
    // get next axis
    index = (index + 1) % dimensions;
    nearest_(less_than ? root->left_ : root->right_, point, index);
    // check hypersphere intersection
    if (dx * dx >= best_distance_) {
        // we can ignore the other subtree as the hypersphere does not intersect.
        return;
    }
    // we have to check the other subtree as well
    nearest_(less_than ? root->right_ : root->left_, point, index);
}

}  // namespace kdtree
