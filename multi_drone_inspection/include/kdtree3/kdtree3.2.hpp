#ifndef _MULTI_DRONE_INSPECTION_KDTREE3_2_HPP_
#define _MULTI_DRONE_INSPECTION_KDTREE3_2_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <execution>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
#include <tuple>
#include <variant>
#include <vector>

#include "kdtree3/utils.hpp"

namespace kdtree {

// template <typename T>
// using option = std::optional<T>;
using vec3 = Eigen::Vector3f;
using node = std::optional<vec3>;
using std::size_t;
// root at [1]
// remember to use [0]
// when do we rebalance 100, 500, 1000
class kdtree3 {
   public:
    kdtree3(const std::vector<vec3>& points);
    auto insert(const vec3& point) -> void { insert(point, 1); }
    auto contains(const vec3& point) -> bool { return contains(point, 1); }
    auto nearest_neighbor(const vec3& point) -> std::optional<vec3>;
    auto bft(std::function<void(const vec3& point, const size_t depth)> f,
             const bool left_to_right = true) const -> void;
    auto max_depth() -> size_t;
    auto is_balanced() -> bool { return max_depth() < std::log2(n_nodes_); }

   private:
    auto insert(const vec3& point, const size_t idx) -> void {}
    auto contains(const vec3& point, const size_t idx) -> bool {}

    inline auto left(const size_t idx) const -> size_t { return 2 * idx; }
    inline auto right(const size_t idx) const -> size_t { return 2 * idx + 1; }
    inline auto parent(const size_t idx) const -> size_t {
        return idx - (idx % 2 == 0 ? 0 : 1) / 2;
    }
    inline auto depth(const size_t idx) const -> size_t { return std::floor(std::log2(idx)); }
    inline auto squared_distance(const vec3& v1, const vec3& v2) const -> double {
        using kdtree::utils::square;
        return square(v1.x() - v2.x()) + square(v1.y() - v2.y()) + square(v1.z() - v2.z());
    }
    /**
     * @brief less than is left greater than is right
     *
     * @param n1
     * @param n2
     * @param depth
     * @return true
     * @return false
     */
    auto compare_nodes_based_on_depth(const vec3& n1, const vec3& n2, const size_t depth) const
        -> bool {
        switch (depth % 3) {
            case 0:
                return n1.x() < n2.x();
                break;
            case 1:
                return n1.y() < n2.y();
                break;
            case 2:
                return n1.z() < n2.z();
                break;
        }
    }

    auto rebalance() -> void;
    auto resize() -> void;
    auto construct_kdtree_recursive(std::vector<vec3> v, const size_t idx, const size_t depth)
        -> void;

    struct dim_comparator {
        virtual auto operator()(const vec3& v1, const vec3& v2) const -> bool;
    };

    struct compare_along_x_dim : public dim_comparator {
        auto operator()(const vec3& v1, const vec3& v2) const -> bool override {
            return v1.x() < v2.x();
        }
    };

    struct compare_along_y_dim : public dim_comparator {
        auto operator()(const vec3& v1, const vec3& v2) const -> bool override {
            return v1.y() < v2.y();
        }
    };

    struct compare_along_z_dim : public dim_comparator {
        auto operator()(const vec3& v1, const vec3& v2) const -> bool override {
            return v1.z() < v2.z();
        }
    };

    auto generate_dim_comparator(const size_t dim) -> dim_comparator {
        switch (dim % 3) {
            case 0:
                return compare_along_x_dim();
            case 1:
                return compare_along_y_dim();
            case 2:
                return compare_along_z_dim();
        }
    }

    size_t n_nodes_ = 0;
    std::vector<node> tree_{};
};

}  // namespace kdtree

#endif  // _MULTI_DRONE_INSPECTION_KDTREE3_2_HPP_
