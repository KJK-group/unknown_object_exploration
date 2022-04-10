#include "multi_drone_inspection/rrt/rrt.hpp"

#include <numeric>

#include "multi_drone_inspection/rrt/rrt_builder.hpp"
#include "multi_drone_inspection/utils/random.hpp"

namespace mdi::rrt {

auto RRT::run() -> std::optional<std::vector<vec3>> {
    for (std::size_t i = 0; i < max_iterations_; ++i) {
        const auto random_point = sample_random_point();
        auto& nearest_neighbor = get_nearest_neighbor(random_point);

        const auto vector_from_nearest_neighbor_to_random_point =
            random_point - nearest_neighbor.position;

        // TODO: use voxblox to check for valid raycast
        // vector_from_nearest_neighbor_to_random_point.normalize();
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " "
                  << vector_from_nearest_neighbor_to_random_point.normalized() << std::endl;
        ;

        const auto point_of_new_node =
            vector_from_nearest_neighbor_to_random_point.normalized() * step_size_ +
            nearest_neighbor.position;
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << point_of_new_node
                  << std::endl;

        nodes_.emplace_back(node{point_of_new_node, &nearest_neighbor});
        auto& new_node = nodes_.back();
        // auto new_node = node{random_point, &nearest_neighbor};
        nearest_neighbor.children.push_back(&new_node);
        // FIXME: this creates a copy of new_node, maybe std::move fixes it ???
        // nodes_.push_back(std::move(new_node));

        std::for_each(on_new_node_created_cb_list.begin(), on_new_node_created_cb_list.end(),
                      [=](const auto& cb) { cb(nearest_neighbor.position, new_node.position); });

        if ((new_node.position - goal_position_).norm() <= max_dist_goal_tolerance_) {
            std::for_each(on_goal_reached_cb_list.begin(), on_goal_reached_cb_list.end(),
                          [&](const auto& cb) { cb(new_node.position, nodes_.size()); });
            // we did it reddit
            node* ptr = &new_node;
            auto solution_waypoint_path = std::vector<vec3>();
            while (ptr->parent != nullptr) {
                solution_waypoint_path.push_back(ptr->position);
                ptr = ptr->parent;
            }
            return solution_waypoint_path;
        }
    }

    return std::nullopt;
}

auto RRT::sample_random_point() -> vec3 {
    return sampling_radius_ * mdi::utils::random::sample_random_point_inside_unit_sphere() +
           start_position_;
}

auto RRT::get_nearest_neighbor(const vec3& point) -> RRT::node& {
    std::size_t index{};
    auto shortest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        const auto& node = nodes_[i];
        auto distance = (node.position - point).norm();
        if (distance < shortest_distance) {
            shortest_distance = distance;
            index = i;
        }
    }
    return nodes_[index];
}

auto RRT::builder() -> RRTBuilder { return RRTBuilder(); }

}  // namespace mdi::rrt
