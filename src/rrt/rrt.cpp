#include "multi_drone_inspection/rrt/rrt.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
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

auto RRT::sample_random_point() -> vec3 {
    auto random_pt =
        rng_.sample_random_point_inside_unit_sphere(direction_from_start_to_goal_, goal_bias_);
    return sampling_radius_ * random_pt + start_position_;
}

auto RRT::find_nearest_neighbor(const vec3& point) -> RRT::node* {
    std::size_t index{0};
    auto shortest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        auto distance = (nodes_[i].position_ - point).norm();
        if (distance < shortest_distance) {
            shortest_distance = distance;
            index = i;
        }
    }
    return &nodes_[index];
}

auto RRT::bft_(const std::function<void(const vec3& pt, const vec3& parent_pt)>& f) const -> void {
    auto queue = std::queue<const node*>{};
    queue.push(&nodes_[0]);
    while (! queue.empty()) {
        auto n = queue.front();
        queue.pop();
        // todo
        // if (n == nullptr) {
        //     continue;
        // }

        // special case is the root node.
        f(n->position_, (n->parent == nullptr ? n->position_ : n->parent->position_));

        if (n->is_leaf()) {
            continue;
        }

        for (const auto child : n->children) {
            queue.push(child);
        }
    }
}

auto RRT::grow_() -> bool {
    if (! (remaining_iterations_ > 0)) {
        return false;
    }

    const auto t_start = std::chrono::high_resolution_clock::now();

    const auto random_pt = sample_random_point();
    auto nearest_neighbor = find_nearest_neighbor(random_pt);

    // this horrow is needed to avoid a race condition with Eigen that
    // causes the vector to be overwritten with garbage value.
    const auto [x, y, z] = [=, &nearest_neighbor]() {
        const auto& pt = nearest_neighbor->position_;
        const auto direction = (random_pt - pt).normalized();
        const auto new_pt = pt + direction * step_size_;
        return std::make_tuple(new_pt.x(), new_pt.y(), new_pt.z());
    }();

    Eigen::Vector3f v;
    v << x, y, z;

    // TODO: use voxblox to check for valid raycast
    const auto line_between_random_point_and_its_nearest_neighbor_is_free_space = true;
    if (! line_between_random_point_and_its_nearest_neighbor_is_free_space) {
        return false;
    }

    // add new node to tree, and update pointers.
    nodes_.emplace_back(v, nearest_neighbor);
    auto& new_node = nodes_.back();
    nearest_neighbor->children.push_back(&new_node);

    const auto t_end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    timimg_measurements_.push_back(duration);

    call_cbs_for_event_on_new_node_created(nearest_neighbor->position_, new_node.position_);

    --remaining_iterations_;

    const auto reached_goal =
        (new_node.position_ - goal_position_).norm() <= max_dist_goal_tolerance_;

    if (reached_goal) {
        // a path has been found from the start coordinate to the goal coordinate,
        // so we need to backtrack to the start coordinate, to get the path as
        // a list of coordinates.
        call_cbs_for_event_on_goal_reached(new_node.position_);

        // clear any previous waypoints found.
        if (! waypoints_.empty()) {
            waypoints_.clear();
        }
        // backtrack to get waypoints
        node* ptr = &new_node;
        do {
            waypoints_.push_back(ptr->position_);
            ptr = ptr->parent;
        } while (ptr != nullptr);

        return true;
    }

    // test direct path to goal
    const auto test_full_edge_from_new_point_to_goal =
        probability_of_testing_full_path_from_new_node_to_goal_ > rng_.random01();
    call_cbs_for_event_on_trying_full_path();
    if (test_full_edge_from_new_point_to_goal) {
        const auto edge = (goal_position_ - new_node.position_);
        // TODO: do raycast
        const auto edge_is_valid = true;
        if (edge_is_valid) {
            // add new node to tree, and update pointers.
            nodes_.emplace_back(goal_position_, &new_node);
            // got this far
            auto& new_node = nodes_.back();
            nearest_neighbor->children.push_back(&new_node);

            // TODO: refactor to make DRY
            // clear any previous waypoints found.
            if (! waypoints_.empty()) {
                waypoints_.clear();
            }
            // backtrack to get waypoints
            node* ptr = &new_node;
            do {
                waypoints_.push_back(ptr->position_);
                ptr = ptr->parent;
            } while (ptr != nullptr);

            return true;
        }
    }

    return false;
}

auto RRT::call_cbs_for_event_on_new_node_created(const vec3& parent_pt, const vec3& new_pt) const
    -> void {
    std::for_each(on_new_node_created_cb_list.begin(), on_new_node_created_cb_list.end(),
                  [&](const auto& cb) { cb(parent_pt, new_pt); });
}

auto RRT::call_cbs_for_event_on_goal_reached(const vec3& pt) const -> void {
    std::for_each(on_goal_reached_cb_list.begin(), on_goal_reached_cb_list.end(),
                  [&](const auto& cb) { cb(pt, nodes_.size()); });
}

auto RRT::call_cbs_for_event_on_clearing_nodes_in_tree() const -> void {
    std::for_each(on_clearing_nodes_in_tree_cb_list.begin(),
                  on_clearing_nodes_in_tree_cb_list.end(), [&](const auto& cb) { cb(); });
}

auto RRT::builder() -> RRTBuilder { return RRTBuilder(); }

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
}  // namespace mdi::rrt

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
    os << "  number_of_nodes: " << rrt.get_number_of_nodes() << '\n';
    const auto connectivity = rrt.connectivity();
    os << "  connectivity: " << connectivity << '\n';
    const auto fully_connected = connectivity == rrt.size();
    os << "  fully_connected: " << (fully_connected ? "true" : "false") << '\n';
    return os;
}

}  // namespace mdi::rrt
