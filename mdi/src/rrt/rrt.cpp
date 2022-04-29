#include "mdi/rrt/rrt.hpp"

#include <fmt/core.h>
#include <ros/console.h>
#include <unistd.h>

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
#include <stack>
#include <tuple>
#include <utility>
#include <vector>

#include "mdi/rrt/rrt_builder.hpp"
#include "mdi/utils/eigen.hpp"
#include "mdi/utils/random.hpp"
#include "mdi/utils/rosparam.hpp"
#include "ros/param.h"

namespace mdi::rrt {

auto RRT::run() -> std::optional<waypoints_type> {
    // TODO: enable
    // if (collision_free_(start_position_, goal_position_)) {
    //     // we might get lucky here, and a direct path between start and goal exists :-)
    //     waypoints_.push_back(goal_position_);
    //     waypoints_.push_back(start_position_);
    //     return {waypoints_};
    // }

    while (remaining_iterations_ > 0) {
        if (grow_()) {
            return {waypoints_};
        }
    }

    return {};
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

RRT::RRT(const vec3& start_position, const vec3& goal_position, float step_size, float goal_bias,
         std::size_t max_iterations, float max_dist_goal_tolerance,
         float probability_of_testing_full_path_from_new_node_to_goal) {
    // : octomap_(std::make_unique<mdi::Octomap>()) {
    {
        start_position_ = start_position;
        goal_position_ = goal_position;
        const auto direction = goal_position - start_position;
        direction_from_start_to_goal_ = direction;
        // TODO: figure out the best radius
        sampling_radius_ = direction.norm() * 1.5;
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
        auto err_msg = "goal_bias must be in the range [0., 1.] but was " + std::to_string(goal_bias);
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

    probability_of_testing_full_path_from_new_node_to_goal_ = probability_of_testing_full_path_from_new_node_to_goal;
}

auto RRT::sample_random_point_() -> vec3 {
    auto random_pt = rng_.sample_random_point_inside_unit_sphere(direction_from_start_to_goal_, goal_bias_);
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
        // TODO: would be better not to expose the underlying array here.
        for (const auto& kdtree : bucket.forest) {
            if (kdtree != nullptr) {
                if (const auto opt = kdtree->nearest(pt)) {
                    const auto [_, shortest_distance, index] = opt.value();
                    // const auto distance = kdtree->distance();
                    nearest_neighbor_candidates.push_back(nearest_neighbor_candidate_t{shortest_distance, index});
                }
            }
        }
    }
#endif  // USE_KDTREE

    // 2. find nearest neighbor in the linear search segment.
    nearest_neighbor_candidates.push_back([&] {
        std::size_t index{0};
        auto shortest_distance = std::numeric_limits<double>::max();
        for (std::size_t i = linear_search_start_index_; i < nodes_.size(); ++i) {
            // TODO: use squared distance instead of sqrt distance. as the size is not of interest, but only relative
            // ordering of distances between points.
            const auto distance = (nodes_[i].position_ - pt).norm();
            if (distance < shortest_distance) {
                shortest_distance = distance;
                index = i;
            }
        }
        return nearest_neighbor_candidate_t{shortest_distance, index};
    }());

    // 3. compare all candidates and select the nearest one.
    const auto index = [&] {
        const auto smaller_distance = [](const auto& c1, const auto& c2) { return c1.distance < c2.distance; };
        std::sort(nearest_neighbor_candidates.begin(), nearest_neighbor_candidates.end(), smaller_distance);

        return nearest_neighbor_candidates.front().index;
    }();

    return &nodes_[index];
}

auto RRT::bft_(const std::function<void(const vec3& parent_pt, const vec3& child_pt)>& f, bool skip_root) const
    -> void {
    const auto root = &nodes_[0];
    assert(root != nullptr);
    // handle special case when node is the root.
    if (not skip_root) {
        f(root->position_, root->position_);
    }

    auto queue = std::queue<const node_t*>{};
    for (const auto child : root->children) {
        queue.push(child);
    }

    while (! queue.empty()) {
        const auto node = queue.front();
        queue.pop();

        f(node->parent->position_, node->position_);

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
    --remaining_iterations_;

#ifdef USE_KDTREE
    {
        // determine if kdtrees should be updated. --------------------------------
        const auto n = static_cast<int>(nodes_.size());  // number of nodes

        const auto number_of_nodes_not_in_kdtrees = n - n_kdtree_nodes_;
        assert(number_of_nodes_not_in_kdtrees >= 0);
        const auto update_kdtrees = number_of_nodes_not_in_kdtrees == max_number_of_nodes_to_do_linear_search_on_;

        if (update_kdtrees) {
            ROS_INFO_STREAM("update kdtrees");

            auto bucket_sizes = std::vector<int>();
            std::transform(kdtree3s_.begin(), kdtree3s_.end(), std::back_inserter(bucket_sizes),
                           [](const auto& bucket) { return bucket.number_of_trees(); });
            // make a copy of the buckets to compare with later to compute diff.
            auto bucket_sizes_before = bucket_sizes;

            const auto bucket_is_full = [](int bucket_size) {
                return bucket_size == max_number_of_kdtrees_per_bucket_;
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
                // iterate through each bucket except the last one, and propagate the
                // change in sizes.
                for (auto i = 0; i < bucket_sizes.size() - 1; ++i) {
                    if (bucket_is_full(bucket_sizes[i])) {
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
            const auto bucket_sizes_diff = [&]() {
                auto& bucket_sizes_updated = bucket_sizes;
                const auto a_new_bucket_has_been_added = bucket_sizes_before.size() < bucket_sizes_updated.size();
                // append 0, to make difference easier to calculate.
                if (a_new_bucket_has_been_added) {
                    ROS_INFO_STREAM("a new bucket has been added");
                    bucket_sizes_before.push_back(0);
                }
#ifndef NDEBUG  // this object macro is defined by cpp spec when debug build is selected
                std::cout << "before\tafter\tdiff" << '\n';
#endif  // NDEBUG
                auto result = std::vector<int>(bucket_sizes_updated.size());
                // subtract each bucket element wise
                for (auto i = 0; i < result.size(); ++i) {
                    const auto before = bucket_sizes_before[i];
                    const auto after = bucket_sizes_updated[i];
                    const auto diff = after - before;
                    result[i] = diff;
#ifndef NDEBUG  // this object macro is defined by cpp spec when debug build is selected
                    std::cout << before << '\t' << after << '\t' << diff << '\n';
#endif  // NDEBUG
                }

                return result;
            }();

            // TEST:
            const auto create_larger_bucket = bucket_sizes_diff.size() > kdtree3s_.size();
            if (create_larger_bucket) {
                add_bucket();
            }
            // apply the diff
            for (auto i = 0; i < bucket_sizes_diff.size(); ++i) {
                const auto diff = bucket_sizes_diff[i];
                auto& bucket = kdtree3s_[i];
                if (diff < 0) {
                    // deallocate trees
                    bucket.delete_trees();
                    ROS_INFO("delete kdtrees in bucket %d and merge them into a kdtree in bucket %d", i, i + 1);
                } else if (diff > 0) {
                    // create tree
                    ROS_INFO("create kdtree in bucket %d", i);

                    // compute interval [start, end)
                    const auto [start, end] = [&] {
                        auto total = 0;
                        for (int i = 0; i < bucket_sizes.size(); ++i) {
                            const auto trees_in_bucket = bucket_sizes[i];
                            const auto& b = kdtree3s_[i];
                            total += trees_in_bucket * b.size_of_a_tree;
                        }
                        const auto offset = bucket.size_of_a_tree;
                        // e.g. total = 1000, and the size of the bucket is 1000, so
                        // start = 0, and end = 1000 to get the interval.
                        return std::make_pair(total - offset, total);
                    }();
                    // create generator object, that transform existing nodes in
                    // the interval, into the format used by a kdtree.
                    auto generator = [this, start = start, end = end] {
                        assert(start < end);
                        assert(0 <= start && start < nodes_.size());
                        assert(0 <= end && end < nodes_.size());
                        auto it = std::next(nodes_.begin(), start);
                        // the lambda needs to bo mutable because we want a different output for each invocation.
                        // Otherwise the same output will be generated.
                        return [it, index = start]() mutable -> std::pair<kdtree3::point_type, std::size_t> {
                            // get point and advance iterator
                            const auto pos = (*it++).position_;
                            return {pos, index++};
                        };
                    }();

                    bucket.add_kdtree(generator, end - start);

                    // recompute number of kdtree nodes.
                    // its correct to only recompute it here because the number of kdtree nodes
                    // only changes when a new kdtree is created,
                    // and only one kdtree at max can be created when this method is called.
                    n_kdtree_nodes_ =
                        std::transform_reduce(kdtree3s_.begin(), kdtree3s_.end(), 0, std::plus<>(),
                                              [](const auto& bucket) { return bucket.number_of_nodes_in_bucket(); });
                    // update the index from where the linear search should start. we
                    // do not do linear search among the nodes with a kdtree index, so it
                    // should start at the end of the kdtree indexes.
                    linear_search_start_index_ = end;
                }
            }
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

    // TODO: decrease radius
    const auto random_pt = sample_random_point_();
    auto v_near = find_nearest_neighbor_(random_pt);
    auto x_min = (*v_near).position_;

    // this horror is needed to avoid a race condition with Eigen that
    // causes the vector to be overwritten with garbage values.
    const auto [x, y, z] = [=, &v_near]() {
        const auto& pt = v_near->position_;
        const auto direction = (random_pt - pt).normalized();
        const auto new_pt = pt + direction * step_size_;
        return std::make_tuple(new_pt.x(), new_pt.y(), new_pt.z());
    }();

    Eigen::Vector3f x_new;
    x_new << x, y, z;

    // const auto edge_between_u_and_v_intersects_a_occupied_voxel = [this](const auto& u, const auto& v) {
    //     const auto convert_to_pt = [](const auto& v) -> mdi::Octomap::point_type { return {v.x(), v.y(), v.z()}; };
    //     // if opt is the some variant, then it means that a occupied voxel was hit.
    //     // TODO: handle this case in an other way
    //     if (octomap_ == nullptr) {
    //         return false;
    //     }
    //     const auto opt = octomap_->raycast(convert_to_pt(u), convert_to_pt(v));
    //     return opt.has_value();
    // };

    const auto edge_between_u_and_v_is_free = [this](const auto& u, const auto& v) -> bool {
        return collision_free_(u, v);
    };

    const auto edge_between_u_and_v_intersects_a_occupied_voxel = [this](const auto& u, const auto& v) {
        return ! collision_free_(u, v);
    };

    if (edge_between_u_and_v_intersects_a_occupied_voxel((*v_near).position_, x_new)) {
        return false;
    }

    auto& inserted_node = insert_node_(x_new, v_near);

    // auto c_min = cost(x_min) + line(v_near, x_new);

    // auto c_new = 2;
    // if (c_new < c_min) {
    //     // if collision_free(v_near, x_new)
    //     // x_min = v_near
    //     // c_min = c_near
    // }

#ifdef MEASURE_PERF
    const auto t_end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start);
    timimg_measurements_.push_back(duration);
#endif

    call_cbs_for_event_on_new_node_created_(v_near->position_, inserted_node.position_);

    const auto reached_goal = (inserted_node.position_ - goal_position_).norm() <= max_dist_goal_tolerance_;

    if (reached_goal) {
        // try direct edge when within tolerance to goal_position_
        if (edge_between_u_and_v_is_free(inserted_node.position_, goal_position_)) {
            auto& new_node = insert_node_(goal_position_, &inserted_node);
            call_cbs_for_event_on_new_node_created_(inserted_node.position_, goal_position_);
            // a path has been found from the start coordinate to the goal coordinate,
            // so we need to backtrack to the start coordinate, to get the path as
            // a list of coordinates.
            // auto& last_node = nodes_.back();
            call_cbs_for_event_on_goal_reached_(new_node.position_);
            backtrack_and_set_waypoints_starting_at_(&new_node);

            return true;
        }
    }

    // test direct path to goal
    const auto test_full_edge_from_new_point_to_goal =
        probability_of_testing_full_path_from_new_node_to_goal_ > rng_.random01();
    if (test_full_edge_from_new_point_to_goal) {
        call_cbs_for_event_on_trying_full_path_(inserted_node.position_, goal_position_);
        if (edge_between_u_and_v_is_free(inserted_node.position_, goal_position_)) {
            auto& new_node = insert_node_(goal_position_, &inserted_node);
            backtrack_and_set_waypoints_starting_at_(&new_node);

            return true;
        }
    }

    return false;
}

// TODO: discard edges which gives a sharp turning angle
auto RRT::optimize_waypoints_() -> void {
    // cannot optimize
    if (waypoints_.size() < 2) {
        return;
    }

    std::cout << "[INFO] optimize_waypoints ..." << '\n';

    // use euclidean distance for cost
    const auto cost = [this](const std::size_t w1_index, const std::size_t w2_index) {
        assert(0 <= w1_index && w1_index < waypoints_.size());
        assert(0 <= w2_index && w2_index < waypoints_.size());
        assert(w1_index != w2_index);
        auto d = (waypoints_[w1_index] - waypoints_[w2_index]).norm();
        return d;
    };

    struct edge {
        edge(std::size_t f, std::size_t t) : from(f), to(t) {}
        std::size_t from, to;
    };

    const auto edge_is_collision_free = [this](const edge& e) -> bool {
        // TODO: raycast
        const auto [from, to] = e;
        assert(0 <= from && from < waypoints_.size());
        assert(0 <= to && to < waypoints_.size());

        return collision_free_(waypoints_[from], waypoints_[to]);
        // return true;
    };

    // create a range [0, |W| - 1]
    auto indices = std::vector<std::size_t>(waypoints_.size());
    std::iota(indices.begin(), indices.end(), 0);

    const auto w_goal = waypoints_.size() - 1;
    const auto w_start = 0;

    auto solution = std::stack<edge>();
    auto s = std::stack<edge>();
    s.emplace(w_start, w_start);

    // distances does not change so we compute them once
    auto distances_to_w_goal = std::vector<float>();
    std::transform(indices.cbegin(), indices.cend(), std::back_inserter(distances_to_w_goal),
                   [&](const auto i) { return cost(i, w_goal); });
    // TODO: not optimal
    while (! s.empty()) {
        const auto [w_parent, w] = s.top();
        if (w == w_goal) {
            break;
        }
        s.pop();

        const auto search_radius = cost(w, w_goal);

        const auto within_relevant_search_radius = [&distances_to_w_goal, search_radius](const std::size_t w_neighbor) {
            return distances_to_w_goal[w_neighbor] <= search_radius;
        };

        // TODO: maybe filter on turning angle
        // w_neighbors <- { w_neighbor in W | cost(w, w_neighbor) <= search_radius /\ edge_is_collision_free(w,
        // w_neighbor) }
        auto w_neighbors = std::vector<std::size_t>();
        std::copy_if(indices.cbegin(), indices.cend(), std::back_inserter(w_neighbors),
                     [&](const std::size_t w_neighbor) {
                         return within_relevant_search_radius(w_neighbor) && edge_is_collision_free({w, w_neighbor});
                     });

        // reject solution node if there are no valid neighbors
        if (w_neighbors.empty()) {
            solution.pop();
            continue;
        }

        // sort based on highest cost since we insert into a stack, and want the lowest cost first.
        std::sort(w_neighbors.begin(), w_neighbors.end(), [&](const std::size_t w1, const std::size_t w2) {
            return distances_to_w_goal[w1] > distances_to_w_goal[w2];
        });

        for (const auto w_neighbor : w_neighbors) {
            s.emplace(w, w_neighbor);
        }

        solution.push(s.top());
    }

    if (s.empty()) {
        std::cout << "nothing to do  ¯\\_(ツ)_/¯" << '\n';
        return;  // nothing to do  ¯\_(ツ)_/¯
    }
    std::vector<std::size_t> solution_indices = {w_start};

    while (! solution.empty()) {
        const auto [_, w] = solution.top();
        solution.pop();
        solution_indices.push_back(w);
    }

    // TODO: check if this is necessary
    if (solution_indices.back() != w_goal) {
        solution_indices.push_back(w_goal);
    }

    auto solution_waypoints = std::vector<waypoint_type>();
    for (const auto i : solution_indices) {
        solution_waypoints.push_back(waypoints_[i]);
    }

    // TODO: interpolate points a long path so bezier spline interpolation is better
    waypoints_.clear();
    waypoints_ = solution_waypoints;

    std::cout << "[INFO] optimize_waypoints ... DONE" << '\n';

    // TODO: maybe do start -> goal and goal -> start, and compare them

    // const auto cost = [](const coordinate_type& a, const coordinate_type& b) { return (a - b).norm(); };
    // // no need to use this because of the Cauchy Schwartz inequality
    // // https://en.wikipedia.org/wiki/Cauchy%E2%80%93Schwarz_inequality
    // const auto better = [cost](const coordinate_type& a, const coordinate_type& b, const coordinate_type& c) {
    //     // triangle inequality
    //     // the direct path between a and c i.e. a -> c, is better than a -> b -> c
    //     return cost(a, c) <= cost(a, b) + cost(b, c);
    // };

    // const auto minimal_cost = cost(start_position_, goal_position_);

    // goal -> start
    // {0, 2}
    // {0, 3}
    // {0, 4}
    // {0, 5}
    // {0, 6}
    // {0, 7}
    // {0, 8}
    // {8, 10}
    // {8, 11}
    // partition on .from, max on .to
    // {0, 8} -> {8, 11}
    // optimized path is then {waypoints_[0], waypoints_[8], waypoints_[11]}
    // start -> goal
    // {11, 9}
    // {11, 8}
    // {11, 7}
    // {11, 6}
    // {6, 4}
    // {6, 3}
    // {6, 2}
    // {6, 1}
    // {6, 0}
    // partition on .from, max on .to
    // {11, 6} -> {6, 0}
    // optimize path is then  {waypoints_[11], waypoints_[6], waypoints_[0]}
    // return min(cost({waypoints_[11], waypoints_[6], waypoints_[0]}), cost({waypoints_[0], waypoints_[8],
    // waypoints_[11]}))s

    // const auto optimize =
    //     [this, &](waypoints_type::iterator&& begin, waypoints_type::iterator&& end) {
    //         auto it = begin;
    //         auto offset = 1;  // offset = 2, makes edge cases harder to handle
    //         std::size_t from = *it;
    //         std::size_t to = (*it + offset);
    //         edge e = {*it, (*it + offset)};

    //         do {
    //             if (! edge_is_collision_free({from, to})) {
    //                 edges.emplace_back(from, to - 1);  // the previous to must be valid
    //                 from = to - 1;
    //             }
    //             to = to + 1;
    //         } while (to != *end);

    //         std::stable_sort(edges.begin(), edges.end(), [](const auto& a, const auto& b) { return a.from < b.from;
    //         });
    //         // auto it = std::adjacent_find(edges.begin(), edges.end(), std::less<std::size_t>());

    //         auto spikes = std::vector<std::size_t>(edges.size());
    //         std::adjacent_difference(edges.begin(), edges.end(), spikes.begin(),
    //                                  [](const edge& a, const edge& b) { return a.from - b.from; });

    //         auto ind = std::vector<std::size_t>();
    //         ind.push_back(0);
    //         for (auto i = 0; i < spikes.size(); ++i) {
    //             if (spikes[i] != 0) {
    //                 ind.push_back(i);
    //             }
    //         }
    //     }

    //  max on .to

    // partition

    // loop until edge{_, .to == *end}

    // return edges;
};

auto RRT::backtrack_and_set_waypoints_starting_at_(node_t* start_node) -> bool {
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

    const auto should_optimize_waypoints = true;
    if (should_optimize_waypoints) {
        optimize_waypoints_();
    }

    return true;
}

// TODO: implement
auto RRT::collision_free_(const vec3& a, const vec3& b, float x, float y, float z, float padding,
                          float end_of_raycast_padding) const -> bool {
    const auto raycast = [this](const auto& u, const auto& v) {
        const auto convert_to_pt = [](const auto& v) -> mdi::Octomap::point_type { return {v.x(), v.y(), v.z()}; };
        // if opt is the some variant, then it means that a occupied voxel was hit.
        const auto opt = octomap_->raycast(convert_to_pt(u), convert_to_pt(v));
        return opt.has_value();
    };

    // // TODO: change raycast to take origin and direction

    // float x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4;

    // vec3 v1 = {x1, y1, z1};
    // vec3 v2 = {x2, y2, z2};
    // vec3 v3 = {x3, y3, z3};
    // vec3 v4 = {x4, y4, z4};

    // vec3 direction = {1, 0, 0};
    // float max_range = 1.0f;

    // const auto raycast2 = [&, max_range](const auto& v) { return raycast(v, direction, max_range); };

    // // essentially project a plane, where v1, v2, v3, and v4 are the bounding corners of the plane.
    // return raycast2(v1) && raycast2(v2) && raycast2(v3) && raycast2(v4);
    return true;
}

auto RRT::call_cbs_for_event_on_new_node_created_(const vec3& parent_pt, const vec3& new_pt) const -> void {
    std::for_each(on_new_node_created_cb_list.begin(), on_new_node_created_cb_list.end(),
                  [&](const auto& cb) { cb(parent_pt, new_pt); });
}

auto RRT::call_cbs_for_event_on_goal_reached_(const vec3& pt) const -> void {
    std::for_each(on_goal_reached_cb_list.begin(), on_goal_reached_cb_list.end(),
                  [&](const auto& cb) { cb(pt, nodes_.size()); });
}

auto RRT::call_cbs_for_event_on_trying_full_path_(const vec3& new_node, const vec3& goal) const -> void {
    std::for_each(on_trying_full_path_cb_list.begin(), on_trying_full_path_cb_list.end(),
                  [&](const auto& cb) { cb(new_node, goal); });
}

auto RRT::call_cbs_for_event_on_clearing_nodes_in_tree_() const -> void {
    std::for_each(on_clearing_nodes_in_tree_cb_list.begin(), on_clearing_nodes_in_tree_cb_list.end(),
                  [&](const auto& cb) { cb(); });
}

auto RRT::from_builder() -> RRTBuilder { return {}; }

auto RRT::from_rosparam(std::string_view prefix) -> RRT {
    const auto prepend_prefix = [&](std::string_view key) { return fmt::format("{}/{}", prefix, key); };

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

#ifdef MEASURE_PERF

RRT::~RRT() {
    if (log_perf_measurements_enabled_) {
        const auto output_file = [&] {
            const auto file_exists = std::filesystem::is_regular_file(file_path_csv_);
            if (file_exists) {
                std::cerr << "[INFO] "
                          << "the perf output file " << file_path_csv_ << " already exists" << std::endl;

                const auto unix_timestamp = [] {
                    using namespace std::chrono;
                    const auto now = high_resolution_clock::now();
                    return static_cast<std::uint32_t>(duration_cast<seconds>(now.time_since_epoch()).count());
                }();
                const auto fmt_str = file_path_csv_.stem().string() + "-{}" + ".csv";

                return std::filesystem::path(fmt::format(fmt_str, unix_timestamp));
            }

            return file_path_csv_;
        }();

        std::cerr << "[INFO] "
                  << "writing perf measurements to " << output_file << '\n';

        auto file = std::ofstream(output_file);
        if (file.is_open()) {
            file << "t,nodes\n";  // write csv header
            const auto nanoseconds_to_seconds = [](double nanoseconds) { return nanoseconds / 1e9; };
            // write rows
            for (std::size_t i = 0; i < timimg_measurements_.size(); ++i) {
                const auto& measurement = timimg_measurements_[i];
                file << i << "," << nanoseconds_to_seconds(static_cast<double>(measurement.count())) << '\n';
            }
            file.close();
        }
    }
}

#endif  // MEASURE_PERF

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
    const auto reachable_nodes = rrt.reachable_nodes();
    os << "  reachable_nodes: " << reachable_nodes << '\n';
    const auto fully_connected = reachable_nodes == rrt.size();
    os << "  fully_connected: " << (fully_connected ? "true" : "false") << '\n';
    return os;
}

}  // namespace mdi::rrt
