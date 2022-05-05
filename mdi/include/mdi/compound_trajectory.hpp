#ifndef _MDI_TRAJECTORY_HPP_
#define _MDI_TRAJECTORY_HPP_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <numeric>
#include <variant>
#include <vector>

#include "mdi/bezier_spline.hpp"
#include "mdi/linear_trajectory.hpp"
#include "mdi/utils/utils.hpp"

namespace mdi::trajectory {
constexpr auto MARKER_SCALE = 0.1f;

using Trajectory = std::variant<BezierSpline, LinearTrajectory>;

class CompoundTrajectory {
   public:
    CompoundTrajectory(ros::NodeHandle& nh, ros::Rate& rate, std::vector<Eigen::Vector3f> path, bool visualise = false,
                       float marker_scale = MARKER_SCALE);
    auto get_point_at_distance(float distance) -> Eigen::Vector3f;
    auto get_length() -> float;

   private:
    auto visualise(float scale) -> void;
    std::vector<float> distance_lut;
    std::vector<std::vector<Eigen::Vector3f>> sections;
    std::vector<Trajectory> trajectories;
    ros::NodeHandle nh;
    ros::Rate rate;
    ros::Publisher pub_visualisation;
    int seq_marker;
};
}  // namespace mdi::trajectory
#endif  // _MDI_TRAJECTORY_HPP_