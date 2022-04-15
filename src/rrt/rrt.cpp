#include "multi_drone_inspection/rrt/rrt.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <queue>
#include <sstream>
#include <tuple>

#include "Eigen/src/Core/Matrix.h"
#include "multi_drone_inspection/rrt/rrt_builder.hpp"
#include "multi_drone_inspection/utils/eigen.hpp"
#include "multi_drone_inspection/utils/random.hpp"

namespace mdi::rrt {

auto RRT::run() -> std::optional<std::vector<vec3>> {
    using mdi::utils::eigen::format_vector3_as_row_vector;

    while (remaining_iterations_ > 0) {
        if (grow_()) {
            return waypoints_;
        }
    }
    // for (std::size_t i = 0; i < max_iterations_; ++i) {
    // test full path

    // if (const auto random_probability = mdi::utils::random::random01();
    //     probability_of_testing_full_path_from_new_node_to_goal_ > random_probability) {
    //     const auto vector_from_new_node_to_goal = goal_position_ - new_node.position;
    //     std::for_each(on_trying_full_path_cb_list.begin(), on_trying_full_path_cb_list.end(),
    //                   [&](const auto& cb) { cb(new_node.position, goal_position_); });
    //     // TODO: use voxblox to check for valid raycast
    //     const auto direction_is_collision_free = true;

    //     if (direction_is_collision_free) {
    //         nodes_.emplace_back(node{goal_position_, &new_node});
    //         // auto& new_node = nodes_.back();
    //         // auto new_node = node{random_pt, &nearest_neighbor};
    //         nearest_neighbor.children.push_back(&nodes_.back());
    //         std::for_each(on_goal_reached_cb_list.begin(), on_goal_reached_cb_list.end(),
    //                       [&](const auto& cb) { cb(new_node.position, nodes_.size()); });
    //         // we did it reddit
    //         node* ptr = &nodes_.back();
    //         // auto solution_waypoint_path = std::vector<vec3>();
    //         while (ptr->parent != nullptr) {
    //             waypoints_.push_back(ptr->position);
    //             ptr = ptr->parent;
    //         }
    //         return waypoints_;
    //     }
    // }
    // }

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
            frontier_nodes.push_back(n.position);
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

auto RRT::sample_random_point() -> vec3 {
    auto random_pt =
        rng_.sample_random_point_inside_unit_sphere(direction_from_start_to_goal_, goal_bias_);

    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //           << " random_pt" << random_pt << '\n';
    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //           << " sampling_radius_" << sampling_radius_ << '\n';
    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //           << " start_position_" << start_position_ << '\n';

    return sampling_radius_ * random_pt + start_position_;
}

auto RRT::get_nearest_neighbor(const vec3& point) -> RRT::node* {
    std::size_t index{0};
    auto shortest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        auto distance = (nodes_[i].position - point).norm();
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
    while (!queue.empty()) {
        auto n = queue.front();
        queue.pop();
        // todo
        // if (n == nullptr) {
        //     continue;
        // }

        // special case is the root node.
        f(n->position, (n->parent == nullptr ? n->position : n->parent->position));

        if (n->is_leaf()) {
            continue;
        }

        for (const auto child : n->children) {
            queue.push(child);
        }
    }
}

auto RRT::grow_() -> bool {
    if (!(remaining_iterations_ > 0)) {
        return false;
    }

    const auto random_pt = sample_random_point();
    auto nearest_neighbor = get_nearest_neighbor(random_pt);

    // TODO: use voxblox to check for valid raycast
    auto [x, y, z] = [=, &nearest_neighbor]() {
        auto& pt = nearest_neighbor->position;
        // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " nearest_neighbor->position " << nearest_neighbor->position << '\n';

        auto d = (random_pt - pt).normalized();
        // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " normalized distance " << d << '\n';

        auto new_pt = pt + d * step_size_;
        // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " HOW " << new_pt << '\n';

        return std::make_tuple(new_pt.x(), new_pt.y(), new_pt.z());

        // return new_pt;
    }();

    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " .x()" << new_pt.x() << '\n';

    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " FUCCCCCC" << new_pt << '\n';

    // new_pt.normalize();

    // auto new_pt2 = new_pt.normalized();

    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " new point " << new_pt << '\n';

    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " x " << x << " y " << y << " z " << z << '\n';

    Eigen::Vector3f v;
    v << x, y, z;

    nodes_.emplace_back(v, nearest_neighbor);
    auto& new_node = nodes_.back();
    nearest_neighbor->children.push_back(&new_node);
    call_cbs_for_event_on_new_node_created(nearest_neighbor->position, new_node.position);

    --remaining_iterations_;

    const auto reached_goal =
        (new_node.position - goal_position_).norm() <= max_dist_goal_tolerance_;

    if (reached_goal) {
        call_cbs_for_event_on_goal_reached(new_node.position);

        // we did it reddit
        node* ptr = &new_node;
        if (!waypoints_.empty()) {
            waypoints_.clear();
        }
        do {
            waypoints_.push_back(ptr->position);
            ptr = ptr->parent;
        } while (ptr != nullptr);
        // } while (ptr->parent != nullptr);

        // while (ptr->parent != nullptr) {
        //     waypoints_.push_back(ptr->position);
        //     std::cout << *ptr << std::endl;
        //     ptr = ptr->parent;
        // }
        return true;
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
