#ifndef _MULTI_DRONE_INSPECTION_KDTREE_HPP_
#define _MULTI_DRONE_INSPECTION_KDTREE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <optional>
#include <queue>
#include <random>
#include <type_traits>
#include <vector>

#include "kdtree/point.hpp"

namespace kdtree {

/**
 * C++ k-d tree implementation, based on the C version at rosettacode.org.
 */
template <typename coordinate_t, size_t dimensions>
class kdtree {
    static_assert(std::is_floating_point_v<coordinate_t> || std::is_integral_v<coordinate_t>,
                  "coordinate_t must be an integer of floating point");

   public:
    using point_t = point<coordinate_t, dimensions>;

    // remove copy constructor
    kdtree(const kdtree&) = delete;
    // remove copy assignment operator
    kdtree& operator=(const kdtree&) = delete;
    /**
     * Constructor taking a pair of iterators. Adds each
     * point in the range [begin, end) to the tree.
     *
     * @param begin start of range
     * @param end end of range
     */
    template <typename iterator>
    kdtree(iterator begin, iterator end) : nodes_(begin, end) {
        root_ = make_tree(0, nodes_.size(), 0);
    }

    /**
     * Constructor taking a function object that generates
     * points. The function object will be called n times
     * to populate the tree.
     *
     * @param f function that returns a point
     * @param n number of points to add
     */
    template <typename fn>
    kdtree(fn&& f, size_t n) {
        static_assert(std::is_function_v<fn>, "fn must be a callable object");
        nodes_.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            nodes_.push_back(f());
        }
        root_ = make_tree(0, nodes_.size(), 0);
    }

    /**
     * @return true if empty, false otherwise
     */
    auto empty() const -> bool { return nodes_.empty(); }

    /**
     * @return size_t the number of nodes visited by the last call to @ref
     * nearest().
     */
    auto n_visited() const -> size_t { return n_visited_; }

    /**
     * @brief distance between the input point and return value
     * from the last call to nearest().
     */
    auto distance() const -> float { return std::sqrt(best_distance_); }

    /**
     * Finds the nearest point in the tree to the given point.
     * It is not valid to call this function if the tree is empty.
     *
     * @param pt a point
     * @return the nearest point in the tree to the given point
     */
    auto nearest(const point_t& pt) -> std::optional<point_t> {
        if (root_ == nullptr) {
            return std::nullopt;
        }
        best_ = nullptr;
        n_visited_ = 0;
        best_distance_ = 0;
        nearest(root_, pt, 0);
        return best_->point_;
    }

    auto bft(std::function<void(const point_t& pt, size_t depth)> f) -> void {}

   private:
    struct node {
        node(const point_t& pt) : point_(pt), left_(nullptr), right_(nullptr) {}
        coordinate_t get_element_in_dimension(size_t index) const {
            return point_.get_element_in_dimension(index);
        }
        auto distance(const point_t& pt) const -> double { return point_.distance(pt); }
        point_t point_;
        node* left_;
        node* right_;
    };

    node* root_ = nullptr;
    node* best_ = nullptr;
    double best_distance_ = 0;
    size_t n_visited_ = 0;
    std::vector<node> nodes_;

    struct dimension_comparator {
        dimension_comparator(size_t index) : index_(index) {}
        bool operator()(const node& n1, const node& n2) const {
            return n1.point_.get_element_in_dimension(index_) <
                   n2.point_.get_element_in_dimension(index_);
        }
        size_t index_;
    };

    auto make_tree(size_t begin, size_t end, size_t index) -> node* {
        if (end <= begin) {
            return nullptr;
        }
        size_t n = begin + (end - begin) / 2;
        auto itr = nodes_.begin();
        // compute median
        std::nth_element(itr + begin, itr + n, itr + end, dimension_comparator(index));
        index = (index + 1) % dimensions;
        nodes_[n].left_ = make_tree(begin, n, index);
        nodes_[n].right_ = make_tree(n + 1, end, index);
        return &nodes_[n];
    }

    auto nearest(node* root, const point_t& point, size_t index) -> void {
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
        const auto dx =
            root->get_element_in_dimension(index) - point.get_element_in_dimension(index);
        auto less_than = dx > 0;
        // get next axis
        index = (index + 1) % dimensions;
        nearest(less_than ? root->left_ : root->right_, point, index);
        // check hypersphere intersection
        if (dx * dx >= best_distance_) {
            // we can ignore the other subtree as the hypersphere does not intersect.
            return;
        }
        // we have to check the other subtree as well
        nearest(less_than ? root->right_ : root->left_, point, index);
    }
};

}  // namespace kdtree

#endif  // _MULTI_DRONE_INSPECTION_KDTREE_HPP_
