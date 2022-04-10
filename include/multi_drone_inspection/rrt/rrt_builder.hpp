#ifndef _MULTI_DRONE_INSPECTION_RRT_BUILDER_HPP_
#define _MULTI_DRONE_INSPECTION_RRT_BUILDER_HPP_
#include <eigen3/Eigen/Dense>

#include "multi_drone_inspection/rrt/rrt.hpp"

namespace mdi::rrt {

/**
 * @brief builder pattern for RRT class
 */
class RRTBuilder {
   public:
    RRTBuilder& start_and_goal_position(const Eigen::Vector3f& start, const Eigen::Vector3f& goal) {
        rrt_.start_position_ = start;
        rrt_.goal_position_ = goal;
        // the sampling volume should be large enough to contain the goal.
        rrt_.sampling_radius_ = (start - goal).norm() * 2;
        // insert root node
        rrt_.nodes_.emplace_back(RRT::node{start});

        return *this;
    }

    RRTBuilder& step_size(float step_size) {
        if (step_size < 0.f) {
            auto err_msg = "step_size must be greater than 0.f";
            throw std::invalid_argument(err_msg);
        }
        rrt_.step_size_ = step_size;
        return *this;
    }

    RRTBuilder& max_iterations(std::size_t max_iterations) {
        rrt_.max_iterations_ = max_iterations;
        rrt_.nodes_.reserve(max_iterations);
        return *this;
    }
    RRTBuilder& max_dist_goal_tolerance(float max_dist_goal_tolerance) {
        if (max_dist_goal_tolerance < 0.f) {
            auto err_msg = "max_dist_goal_tolerance must be greater than 0.f";
            throw std::invalid_argument(err_msg);
        }
        rrt_.max_dist_goal_tolerance_ = max_dist_goal_tolerance;
        return *this;
    }
    RRTBuilder& goal_bias(float goal_bias) {
        if (!(0 <= goal_bias && goal_bias <= 1.f)) {
            auto err_msg =
                "goal_bias must be in the range [0., 1.] but was " + std::to_string(goal_bias);
            throw std::invalid_argument(err_msg);
        }
        rrt_.goal_bias_ = goal_bias;
        return *this;
    }

    RRTBuilder& on_new_node_created(
        std::function<void(const Eigen::Vector3f& parent_node, const Eigen::Vector3f& new_node)>
            cb) {
        rrt_.on_new_node_created_cb_list.push_back(cb);
        return *this;
    }
    RRTBuilder& on_goal_reached(
        std::function<void(const Eigen::Vector3f& goal_node, std::size_t n_iterations)> cb) {
        rrt_.on_goal_reached_cb_list.push_back(cb);
        return *this;
    }
    auto build() -> RRT { return std::move(rrt_); }

   private:
    RRT rrt_;
};

}  // namespace mdi::rrt

#endif  // _MULTI_DRONE_INSPECTION_RRT_BUILDER_HPP_
