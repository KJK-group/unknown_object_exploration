#include "multi_drone_inspection/rrt.hpp"

#include <fmt/format.h>
#include <ros/ros.h>

#include "multi_drone_inspection/utils/random.hpp"
#include "multi_drone_inspection/voxblox_manager.hpp"

namespace mpi {

RRT::RRT(Vector3f start_pos, Vector3f goal_pos, float step_size, unsigned int max_iterations,
         float goal_bias, float max_dist_goal_tolerance, VoxelGrid voxelgrid)
    : start_pos_(start_pos),
      goal_pos_(goal_pos),
      step_size_(step_size),
      max_iterations_(max_iterations),
      goal_bias_(goal_bias),
      max_dist_goal_tolerance_(max_dist_goal_tolerance),
      voxelgrid_(voxelgrid) {
    if (!(0 <= goal_bias_ && goal_bias_ <= 1.f)) {
        auto err_msg =
            fmt::format("goal_bias must be in the range [0., 1.] but was {}", goal_bias_);
        ROS_ERROR(err_msg);
        throw std::invalid_argument(err_msg);
    }

    if (step_size_ < 0.f) {
        auto err_msg = fmt::format("step_size must be greater than 0.f");
        ROS_ERROR(err_msg);
        throw std::invalid_argument(err_msg);
    }

    if (max_dist_goal_tolerance_ < 0.f) {
        auto err_msg = fmt::format("max_dist_goal must be greater than 0.f");
        ROS_ERROR(err_msg);
        throw std::invalid_argument(err_msg);
    }

    sample_random_point_on_unit_sphere_surface_ = goal_bias > 0.0f
	? [this] {
        auto direction = goal_position - head;
        return utils::random::sample_random_point_on_unit_sphere_surface(direction, goal_bias);
    } :
	[this] {return utils::random::sample_random_point_on_unit_sphere_surface();};
}

auto RRT::run() -> optional<vector<Vector3f>> {
    // ! fix
    for (auto n = 0; n < max_iterations_; n++) {
        /* code */
        // sample random point
        // 2 check collision if using that point
        // 3 if free then grow tree
    }

    return std::nullopt;
}

auto RRT::growN(unsigned int n) -> void {
    auto remaining_iterations = max_iterations_ - size();
    if (n > remaining_iterations) {
        auto err_msg = fmt::format("n = {} exeeds the number of possible iterations remaining = {}",
                                   n, remaining_iterations);
        ROS_ERROR(err_msg);
        throw std::invalid_argument(err_msg);
    }

    for (int i = 0; i < n; ++i) {
        if (grow_()) {
            break;
        }
    }
}

auto RRT::get_head() -> const Vector3f& { return }

auto RRT::distance_from_head_to_goal() -> float { return (goal_position_ - get_head()).norm(); }
auto RRT::is_head_within_tolerance_of_goal() -> bool {
    return distance_from_head_to_goal() < max_dist_goal_tolerance_;
}

auto RRT::grow_() -> bool {
    // get head
    // sample random point in sphere
    // get direction from head position to goal position.
    auto point = sample_random_point_on_unit_sphere_surface_();
    voxelgrid_.if
        // check if the edge from head to point is blocked by a occupied voxel.
        return is_head_within_tolerance_of_goal();
}

}  // namespace mpi
