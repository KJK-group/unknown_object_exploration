#include "multi_drone_inspection/rrt/rrt.hpp"

#include <fmt/core.h>
#include <ros/console.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <queue>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "multi_drone_inspection/rrt/rrt_builder.hpp"
#include "multi_drone_inspection/utils/eigen.hpp"
#include "multi_drone_inspection/utils/random.hpp"
#include "multi_drone_inspection/utils/rosparam.hpp"
#include "ros/param.h"

namespace mdi::rrt {

auto RRT::run() -> std::optional<std::vector<vec3>> {
    using mdi::utils::eigen::format_vector3_as_row_vector;

    while (remaining_iterations_ > 0) {
        if (grow_()) {
            return waypoints_;
        }
    }

    return std::nullopt;
}

auto RRT::growN(int n) -> bool {
    if (n < 0) {
        throw std::invalid_argument("n must be positive");
    }

    while (remaining_iterations_ > 0 && n > 0) {
        if (grow_()) {
            return true;
        }
        --n;
    }
    return false;
}

auto RRT::get_frontier_nodes() const -> std::vector<vec3> {
    auto frontier_nodes = std::vector<vec3>{};
    for (const auto& n : nodes_) {
        if (n.is_leaf()) {
            frontier_nodes.push_back(n.position_);
        }
    }
    return frontier_nodes;
}

auto RRT::bft(const std::function<void(const vec3& pt)>& f) -> void {
    bft_([&](const vec3& pt, const vec3& parent_pt) { f(pt); });
}

auto RRT::bft(const std::function<void(const vec3& pt, const vec3& parent_pt)>& f) -> void {
    bft_([&](const vec3& pt, const vec3& parent_pt) { f(pt, parent_pt); });
}

RRT::RRT(const vec3& start_position, const vec3& goal_position, float step_size, float goal_bias,
         std::size_t max_iterations, float max_dist_goal_tolerance,
         float probability_of_testing_full_path_from_new_node_to_goal) {
    {
        start_position_ = start_position;
        goal_position_ = goal_position;
        const auto direction = goal_position - start_position;
        direction_from_start_to_goal_ = direction;
        sampling_radius_ = direction.norm() * 2;
        nodes_.emplace_back(start_position);
    }

    if (step_size < 0.f) {
        auto err_msg = "step_size must be greater than 0.f";
        throw std::invalid_argument(err_msg);
    }
    step_size_ = step_size;

    max_iterations_ = max_iterations;
    remaining_iterations_ = max_iterations;
    // DO NOT CHANGE THIS, IF A CONSTANT IS NOT ADDED THEN A DOUBLE FREE HAPPENS
    nodes_.reserve(max_iterations + 10);

    if (max_dist_goal_tolerance < 0.f) {
        auto err_msg = "max_dist_goal_tolerance must be greater than 0.f";
        throw std::invalid_argument(err_msg);
    }
    max_dist_goal_tolerance_ = max_dist_goal_tolerance;

    if (! (0 <= goal_bias && goal_bias <= 1.f)) {
        auto err_msg =
            "goal_bias must be in the range [0., 1.] but was " + std::to_string(goal_bias);
        throw std::invalid_argument(err_msg);
    }
    goal_bias_ = goal_bias;

    if (! (0 <= probability_of_testing_full_path_from_new_node_to_goal &&
           probability_of_testing_full_path_from_new_node_to_goal <= 1.f)) {
        auto err_msg =
            "probability_of_testing_full_path_from_new_node_to_goal must be in the range "
            "[0., 1.] but was " +
            std::to_string(probability_of_testing_full_path_from_new_node_to_goal);
        throw std::invalid_argument(err_msg);
    }

    probability_of_testing_full_path_from_new_node_to_goal_ =
        probability_of_testing_full_path_from_new_node_to_goal;
}

auto RRT::sample_random_point_() -> vec3 {
    auto random_pt =
        rng_.sample_random_point_inside_unit_sphere(direction_from_start_to_goal_, goal_bias_);
    return sampling_radius_ * random_pt + start_position_;
}

auto RRT::find_nearest_neighbor_(const vec3& pt) -> RRT::node_t* {
    struct nearest_neighbor_candidate_t {
        double distance = std::numeric_limits<double>::max();
        std::size_t index{};
    };
    using nearest_neighbor_candidates_t = std::vector<nearest_neighbor_candidate_t>;
    auto nearest_neighbor_candidates = nearest_neighbor_candidates_t();

#ifdef USE_KDTREE
    // 1. find nearest neighbor in each kdtree, and store them in a list of candidates.
    for (const auto& bucket : kdtree3s_) {
        for (const auto& kdtree : bucket.forest) {
            // TODO: would be better not to expose the underlying array here.
            if (kdtree != nullptr) {
                if (const auto opt = kdtree->nearest(kdtree3::point_t{pt.x(), pt.y(), pt.z()})) {
                    // TODO: change nearest to also return distance
                    const auto [_, index] = opt.value();
                    const auto distance = kdtree->distance();
                    nearest_neighbor_candidates.push_back(
                        nearest_neighbor_candidate_t{distance, index});
                }
            }
        }
    }
#endif // USE_KDTREE

    // 2. find nearest neighbor in the linear search segment.
    nearest_neighbor_candidates.push_back([&]() {
        std::size_t index{0};
        auto shortest_distance = std::numeric_limits<double>::max();
        for (std::size_t i = linear_search_start_index_; i < nodes_.size(); ++i) {
            auto distance = (nodes_[i].position_ - pt).norm();
            if (distance < shortest_distance) {
                shortest_distance = distance;
                index = i;
            }
        }
        return nearest_neighbor_candidate_t{shortest_distance, index};
    }());

    // 3. compare all candidates and select the nearest one.
    const auto index = [&]() {
        const auto smaller_distance = [](const auto& c1, const auto& c2) {
            return c1.distance < c2.distance;
        };
        // std::less
        std::sort(nearest_neighbor_candidates.begin(), nearest_neighbor_candidates.end(),
                  smaller_distance);

        return nearest_neighbor_candidates.front().index;
    }();

    return &nodes_[index];
}

auto RRT::bft_(const std::function<void(const vec3& pt, const vec3& parent_pt)>& f) const -> void {
    auto queue = std::queue<const node_t*>{};
    queue.push(&nodes_[0]);
    while (! queue.empty()) {
        auto node = queue.front();
        queue.pop();
        // todo
        // if (n == nullptr) {
        //     continue;
        // }

        // special case is the root node.
        f(node->position_, (node->parent == nullptr ? node->position_ : node->parent->position_));

        if (node->is_leaf()) {
            continue;
        }

        for (const auto child : node->children) {
            queue.push(child);
        }
    }
}

/**
 * @brief  add new node to tree, and update pointers.
 * @note only used internally
 * @param pos
 * @param parent
 * @return node& a reference to the new node.
 */
auto RRT::insert_node_(const vec3& pos, node_t* parent) -> node_t& {
    assert(parent != nullptr);
    auto& node = nodes_.emplace_back(pos, parent);
    parent->children.push_back(&node);

#ifdef USE_KDTREE

    {
        // update kdtrees
        // const auto k = static_cast<int>(kdtree3s_.size());  // number of kdtrees
        const auto n = static_cast<int>(nodes_.size());  // number of nodes

        // TODO: cache
        const auto n_kdtree_nodes = std::transform_reduce(
            kdtree3s_.begin(), kdtree3s_.end(), 0, std::plus<>(),
            [](const auto& bucket) { return bucket.number_of_nodes_in_bucket(); });

        const auto number_of_nodes_not_in_kdtrees = n - n_kdtree_nodes;
        assert(number_of_nodes_not_in_kdtrees >= 0);

        // ROS_INFO("number of nodes not in kdtrees: %d", number_of_nodes_not_in_kdtrees);
        const auto update_kdtrees =
            number_of_nodes_not_in_kdtrees == max_number_of_nodes_to_do_linear_search_on_;

        if (update_kdtrees) {
            auto bucket_sizes = std::vector<int>();
            std::transform(kdtree3s_.begin(), kdtree3s_.end(), std::back_inserter(bucket_sizes),
                           [](const auto& bucket) { return bucket.number_of_trees(); });

            auto bucket_sizes_before = bucket_sizes;

            const auto bucket_is_full = [](int bucket_size) {
                // TODO: remove constant
                const auto max_bucket_size = 5;
                return bucket_size == max_bucket_size;
            };
            const auto any_bucket_is_full = [&]() {
                return std::any_of(bucket_sizes.begin(), bucket_sizes.end(), bucket_is_full);
            };

            // always add 1 to the smallest bucket
            bucket_sizes.front() += 1;

            // compute the bucket sizes after updating the kdtrees
            // example: (5 is max bucket size)
            // [3, 0, 0, 0] -> [4, 0, 0, 0]
            // [4, 0, 0, 0] -> [0, 1, 0, 0]
            // [4, 4, 0, 0] -> [0, 0, 1, 0]
            while (any_bucket_is_full()) {
                for (auto i = 0; i < bucket_sizes.size() - 1; ++i) {
                    const auto bucket_size = bucket_sizes[i];
                    if (bucket_is_full(bucket_size)) {
                        bucket_sizes[i] = 0;
                        bucket_sizes[i + 1] += 1;
                    }
                }

                // edge case when the last i.e. largest bucket overflows, then
                // a new bucket has to be created.
                auto& last_bucket_size = bucket_sizes.back();
                if (bucket_is_full(last_bucket_size)) {
                    last_bucket_size = 0;
                    bucket_sizes.emplace_back(1);
                }
            }

            // compute diff
            const auto diff = [&]() {
                const auto a_new_bucket_has_been_added =
                    bucket_sizes_before.size() < bucket_sizes.size();
                // append 0, to make difference easier to calculate.
                if (a_new_bucket_has_been_added) {
                    bucket_sizes_before.push_back(0);
                }

                auto result = std::vector<int>(bucket_sizes.size());
                // subtract each bucket element wise
                for (auto i = 0; i < result.size(); ++i) {
                    result[i] = bucket_sizes[i] - bucket_sizes_before[i];
                }
                return result;
            }();

            // apply the diff
            assert(kdtree3s_.size() == diff.size());
            for (auto i = 0; i < diff.size(); ++i) {
                const auto d = diff[i];
                auto& bucket = kdtree3s_[i];
                if (d < 0) {
                    // deallocate trees
                    bucket.delete_trees();
                    // TODO: somehow set a flag indicating the bucket that overspilled
                } else if (d > 0) {
                    // create tree

                    // compute interval [start, end)
                    const auto start = [&]() {
                        auto sum = 0;
                        for (int i = 0; i < bucket_sizes.size(); ++i) {
                            auto trees_in_bucket = bucket_sizes[i];
                            const auto& b = kdtree3s_[i];
                            std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
                                      << " trees_in_bucket " << trees_in_bucket << " sum " << sum
                                      << '\n';

                            sum += trees_in_bucket * b.size_of_a_tree;
                        }
                        // this is necessary because ...
                        sum -= bucket.size_of_a_tree;
                        return sum;
                    }();

                    std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
                              << " bucket to insert a tree into " << i << '\n';

                    std::cout << "before\tafter\tdiff" << '\n';
                    for (auto i = 0; i < diff.size(); ++i) {
                        std::cout << bucket_sizes_before[i] << '\t' << bucket_sizes[i] << '\t'
                                  << diff[i] << '\n';
                    }

                    // exclusive
                    const auto end = start + bucket.size_of_a_tree;
                    assert(start < end);
                    std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
                              << " start: " << start << " end: " << end << '\n';
                    // create generator object, that transform existing nodes in
                    // the interval, into the format used by a kdtree.
                    auto generator = [this, start, end] {
                        assert(0 <= start && start < nodes_.size());
                        assert(0 <= end && end < nodes_.size());
                        auto it = std::next(nodes_.begin(), start);
                        auto index = start;

                        return [it, index]() mutable -> std::pair<kdtree3::point_t, std::size_t> {
                            const auto& pos = (*it).position_;
                            const auto pt = kdtree3::point_t{pos.x(), pos.y(), pos.z()};
                            std::next(it);
                            // TODO: is index the same for all ???
                            std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
                                      << " index " << index << '\n';

                            return {pt, index++};
                        };
                    }();

                    bucket.add_kdtree(generator, end - start);
                    // TODO: assure this is correct
                    linear_search_start_index_ = end;
                }
            }

            //     auto result = std::vector<int>(bucket_sizes.size());
            //     // a new bucket is created
            //     const auto n = [&]() {
            //         if (copy.size() < bucket_sizes.size()) {
            //             result.back() = 1;
            //             return bucket_sizes.size() - 1;
            //         } else {
            //             return bucket_sizes.size();
            //         }
            //     }();

            //     for (auto i = 0; i < n; ++i) {
            //         d[i] = bucket_sizes[i].instances - copy[i].instances;
            //     }
            //     return result;
            // }
            // ();
            // const auto diff =
            //     // const auto& last_bucket = kdtree3s_.back();
            //     // if (last_bucket.bucket_is_full()) {
            //     //     const auto idx_of_last_bucket = kdtree3s_.size() - 1;
            //     //     // FIXME: remove constants
            //     //     kdtree3s_.emplace_back(1000 *
            //     //                            static_cast<int>(std::pow(5, idx_of_last_bucket)));
            //     // }
            //     kdtree3s_.front().forest

            //     for (const auto& bucket : kdtree3s_) {}

            // // apply diff

            // for (const auto bucket_size : bucket_sizes) {
            // }

            // auto copy = kdtree_size_counts_;

            // kdtree_size_counts_[0].instances += 1;
            // for (auto i = 0; i < kdtree_size_counts_.size(); ++i) {
            //     auto& [size, instances] = kdtree_size_counts_[i];
            //     if (instances == max_number_of_kdtrees_per_bucket_) {
            //         // - 4
            //         // + 1
            //         if (size == kdtree_size_counts_.size() - 1) {
            //             kdtree_size_counts_.emplace_back(size *
            //             max_number_of_kdtrees_per_bucket_, 1); instances = 0;
            //         }
            //     }
            // }

            // // apply diff
            // for (auto i = 0; i < diff.size(); ++i) {
            //     const auto d = diff[i];
            //     if (d < 0) {
            //         // deallocate trees
            //         auto it = kdtree3s_.begin() + (kdtree3s_.size() - d);
            //         std::for_each(it, kdtree3s_.end(), [&](const auto& tree) { delete tree; });
            //         kdtree3s_.erase()
            //     } else if (d > 0) {
            //         // create trees
            //         const auto size = max_number_of_nodes_to_do_linear_search_on_ *
            //                           std::pow(max_number_of_kdtrees_per_bucket_, i);
            //     }
            // }

            // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
            //           << " updating kdtree" << '\n';

            // auto offset = [this, k]() -> std::size_t {
            //     const auto merge_existing_kdtrees_into_one = k == max_number_of_kdtrees_;
            //     if (merge_existing_kdtrees_into_one) {
            //         std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
            //                   << " merge_existing_kdtrees_into_one" << '\n';

            //         std::for_each(kdtree3s_.begin(), kdtree3s_.end(),
            //                       [&](const auto& tree) { delete tree; });
            //         kdtree3s_.clear();
            //         std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
            //                   << " kdtree3s_.size()" << kdtree3s_.size() << '\n';

            //         // update size of kdtrees. 1000 -> 5000 -> 25000
            //         kdtree_size_ = kdtree_size_ * (max_number_of_kdtrees_per_bucket_ + 1);
            //         // TODO: not correct for policy 2
            //         return 0;
            //     } else {
            //         return linear_search_start_index_;
            //     }
            // }();

            // // kdtree3s_.emplace_back(generator, kdtree_size_);
            // kdtree3s_.push_back(new kdtree3(generator, kdtree_size_));
            // // kdtree3s_.push_back(std::make_unique<kdtree3>(generator, kdtree_size_));
            // // kdtree3s_.emplace_front(generator, kdtree_size_);
            // // kdtree3s_.push_front(std::move(std::make_unique<kdtree3>(generator,
            // kdtree_size_)));
            // // advance starting index for linear search
            // linear_search_start_index_ = n;
        }
    }

#endif  // USE_KDTREE

    return node;
}

auto RRT::grow_() -> bool {
    if (! (remaining_iterations_ > 0)) {
        return false;
    }

#ifdef MEASURE_PERF
    const auto t_start = std::chrono::high_resolution_clock::now();
#endif  // MEASURE_PERF

    const auto random_pt = sample_random_point_();
    auto nearest_neighbor = find_nearest_neighbor_(random_pt);

    // this horror is needed to avoid a race condition with Eigen that
    // causes the vector to be overwritten with garbage value.
    const auto [x, y, z] = [=, &nearest_neighbor]() {
        const auto& pt = nearest_neighbor->position_;
        const auto direction = (random_pt - pt).normalized();
        const auto new_pt = pt + direction * step_size_;
        return std::make_tuple(new_pt.x(), new_pt.y(), new_pt.z());
    }();

    Eigen::Vector3f v;
    v << x, y, z;

    auto& inserted_node = insert_node_(v, nearest_neighbor);

    // TODO: use voxblox to check for valid raycast
    const auto line_between_random_point_and_its_nearest_neighbor_is_free_space = true;
    if (! line_between_random_point_and_its_nearest_neighbor_is_free_space) {
        return false;
    }

#ifdef MEASURE_PERF
    const auto t_end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start);
    timimg_measurements_.push_back(duration);
#endif

    call_cbs_for_event_on_new_node_created_(nearest_neighbor->position_, inserted_node.position_);

    --remaining_iterations_;

    const auto reached_goal =
        (inserted_node.position_ - goal_position_).norm() <= max_dist_goal_tolerance_;

    if (reached_goal) {
        // a path has been found from the start coordinate to the goal coordinate,
        // so we need to backtrack to the start coordinate, to get the path as
        // a list of coordinates.
        call_cbs_for_event_on_goal_reached_(inserted_node.position_);
        backtrack_and_set_waypoints_(&inserted_node);

        return true;
    }

    // test direct path to goal
    const auto test_full_edge_from_new_point_to_goal =
        probability_of_testing_full_path_from_new_node_to_goal_ > rng_.random01();
    if (test_full_edge_from_new_point_to_goal) {
        call_cbs_for_event_on_trying_full_path_(inserted_node.position_, goal_position_);

        const auto edge = (goal_position_ - inserted_node.position_);
        // TODO: do raycast
        const auto edge_is_valid = true;
        if (edge_is_valid) {
            auto& new_node = insert_node_(goal_position_, &inserted_node);

            backtrack_and_set_waypoints_(&new_node);

            return true;
        }
    }

    return false;
}

auto RRT::backtrack_and_set_waypoints_(node_t* start_node) -> bool {
    if (start_node == nullptr) {
        return false;
    }
    // clear any previous waypoints found.
    if (! waypoints_.empty()) {
        waypoints_.clear();
    }
    // backtrack to get waypoints
    node_t* ptr = start_node;
    do {
        waypoints_.push_back(ptr->position_);
        ptr = ptr->parent;
    } while (ptr != nullptr);

    return true;
}

auto RRT::call_cbs_for_event_on_new_node_created_(const vec3& parent_pt, const vec3& new_pt) const
    -> void {
    std::for_each(on_new_node_created_cb_list.begin(), on_new_node_created_cb_list.end(),
                  [&](const auto& cb) { cb(parent_pt, new_pt); });
}

auto RRT::call_cbs_for_event_on_goal_reached_(const vec3& pt) const -> void {
    std::for_each(on_goal_reached_cb_list.begin(), on_goal_reached_cb_list.end(),
                  [&](const auto& cb) { cb(pt, nodes_.size()); });
}

auto RRT::call_cbs_for_event_on_trying_full_path_(const vec3& new_node, const vec3& goal) const
    -> void {
    std::for_each(on_trying_full_path_cb_list.begin(), on_trying_full_path_cb_list.end(),
                  [&](const auto& cb) { cb(new_node, goal); });
}

auto RRT::call_cbs_for_event_on_clearing_nodes_in_tree_() const -> void {
    std::for_each(on_clearing_nodes_in_tree_cb_list.begin(),
                  on_clearing_nodes_in_tree_cb_list.end(), [&](const auto& cb) { cb(); });
}

auto RRT::from_builder() -> RRTBuilder { return {}; }

auto RRT::from_rosparam(std::string_view prefix) -> RRT {
    const auto prepend_prefix = [&](std::string_view key) {
        return fmt::format("{}/{}", prefix, key);
    };

    const auto get_int = [&](std::string_view key) {
        auto default_value = int{};
        if (ros::param::get(prepend_prefix(key), default_value)) {
            return default_value;
        }

        const auto error_msg = fmt::format("key {} does not exist in the parameter server.", key);
        throw std::invalid_argument(error_msg);
    };

    const auto get_float = [&](std::string_view key) {
        auto default_value = float{};

        if (ros::param::get(prepend_prefix(key), default_value)) {
            return default_value;
        }

        const auto error_msg = fmt::format("key {} does not exist in the parameter server.", key);
        throw std::invalid_argument(error_msg);
    };

    const auto get_vec3 = [&](std::string_view key) {
        auto default_value = std::vector<float>{};
        std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
                  << " key " << prepend_prefix(key) << '\n';
        if (ros::param::get(prepend_prefix(key), default_value)) {
            assert(default_value.size() == 3);
            const auto x = default_value[0];
            const auto y = default_value[1];
            const auto z = default_value[2];
            return vec3{x, y, z};
        }

        const auto error_msg = fmt::format("key {} does not exist in the parameter server.", key);
        throw std::invalid_argument(error_msg);
    };

    return {get_vec3("start_position"),
            get_vec3("goal_position"),
            get_float("step_size"),
            get_float("goal_bias"),
            static_cast<size_t>(get_int("max_iterations")),
            get_float("max_dist_goal_tolerance"),
            get_float("probability_of_testing_full_path_from_new_node_to_goal")};
}

std::ostream& operator<<(std::ostream& os, const RRT& rrt) {
    os << "RRT:\n";
    os << "  step_size: " << rrt.step_size_ << '\n';
    os << "  goal_bias: " << rrt.goal_bias_ << '\n';
    os << "  probability_of_testing_full_path_from_new_node_to_goal: "
       << rrt.probability_of_testing_full_path_from_new_node_to_goal_ << '\n';
    os << "  max_iterations: " << rrt.max_iterations_ << '\n';
    os << "  remaining_iterations: " << rrt.remaining_iterations_ << '\n';
    os << "  max_dist_goal_tolerance: " << rrt.max_dist_goal_tolerance_ << '\n';
    os << "  sampling_radius: " << rrt.sampling_radius_ << '\n';
    os << "  start_position:\n";
    os << "    x: " << rrt.start_position_.x() << '\n';
    os << "    y: " << rrt.start_position_.y() << '\n';
    os << "    z: " << rrt.start_position_.z() << '\n';
    os << "  goal_position:\n";
    os << "    x: " << rrt.goal_position_.x() << '\n';
    os << "    y: " << rrt.goal_position_.y() << '\n';
    os << "    z: " << rrt.goal_position_.z() << '\n';
    os << "  number_of_nodes: " << rrt.size() << '\n';
    const auto connectivity = rrt.connectivity();
    os << "  connectivity: " << connectivity << '\n';
    const auto fully_connected = connectivity == rrt.size();
    os << "  fully_connected: " << (fully_connected ? "true" : "false") << '\n';
    return os;
}

}  // namespace mdi::rrt
