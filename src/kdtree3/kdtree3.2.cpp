#include "kdtree3/kdtree3.2.hpp"

#include <algorithm>
#include <queue>

#include "kdtree3/utils.hpp"

namespace kdtree {

kdtree3::kdtree3(const std::vector<vec3>& points) {
    tree_.reserve(points.size() * 2);
    auto median = utils::median(points);
}

auto kdtree3::nearest_neighbor(const vec3& point) -> std::optional<vec3> {}

auto kdtree3::bft(std::function<void(const vec3& point, const size_t depth)> f,
                  const bool left_to_right) const -> void {
    using tree_position = std::pair<size_t, size_t>;
    auto indices_to_visit = std::queue<tree_position>();
    inline auto enqueue_indices = [&indices_to_visit](const size_t index, const size_t depth) {
        indices_to_visit.push(std::make_pair(index, depth));
    };
    {
        const auto root_index = 1;
        const auto starting_depth = 1;
        enqueue_indices(root_index, starting_depth);
    }

    auto enqueue_childen = [this, left_to_right, &indices_to_visit, &enqueue_indices]() {
        if (left_to_right) {
            return [this, &indices_to_visit, &enqueue_indices](const size_t index,
                                                               const size_t depth) {
                enqueue_indices(left(index), depth + 1);
                enqueue_indices(right(index), depth + 1);
            };
        } else {
            return [this, &indices_to_visit, &enqueue_indices](const size_t index,
                                                               const size_t depth) {
                enqueue_indices(right(index), depth + 1);
                enqueue_indices(left(index), depth + 1);
            };
        }
    }();

    while (!indices_to_visit.empty()) {
        auto&& [index, depth] = indices_to_visit.front();
        indices_to_visit.pop();
        // check if node is not none i.e. it is not an unused index in the vector
        // which is used for children of leaf nodes.
        if (const auto& opt = tree_.at(index)) {
            const auto& value = *opt;
            f(value, depth);
            enqueue_childen(index, depth + 1);
            // enqueue_indices(left(index), depth + 1);
            // enqueue_indices(right(index), depth + 1);
        }
    }
}

auto kdtree3::max_depth() -> size_t {
    auto depths = std::vector<size_t>();
    bft([&depths](const vec3& point, const size_t depth) { depths.push_back(depth); });

    return std::max_element(depths.begin(), depths.end());
}

auto kdtree3::rebalance() -> void {}

auto kdtree3::resize() -> void {}

auto kdtree3::construct_kdtree_recursive(std::vector<vec3> v, const size_t idx, const size_t depth)
    -> void {
    if (v.size() == 2) {
    } else if (v.size() == 1) {
    }
    // struct dim_comparator {
    //     // dim_comparator(size_t d) : dim(d) {}
    //     size_t dim;
    //     auto operator()(const vec3& v1, const vec3& v2) const -> bool { return v1(dim) < v2(dim);
    //     }
    // };

    const auto dim = depth % 3;
    // auto comparator = dim_comparator{depth % 3};

    // std::sort(v.begin(), v.end(), dim_comparator{depth % 3});
    std::sort(v.begin(), v.end(),
              [=](const vec3& v1, const vec3& v2) { return v1(dim) < v2(dim); });
})
}  // namespace kdtree

}  // namespace kdtree
