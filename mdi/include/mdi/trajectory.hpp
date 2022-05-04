#ifndef _MDI_TRAJECTORY_HPP_
#define _MDI_TRAJECTORY_HPP_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <numeric>
#include <vector>

#include "mdi/bezier_spline.hpp"
#include "mdi/utils/utils.hpp"

namespace mdi {
constexpr auto MARKER_SCALE = 0.1f;
class Trajectory {
   public:
    Trajectory(ros::NodeHandle& nh, ros::Rate& rate, std::vector<Eigen::Vector3f> path,
               float marker_scale = MARKER_SCALE);
    auto get_point_at_distance(float distance) -> Eigen::Vector3f;
    auto get_length() -> float;

   private:
    auto visualise(float scale) -> void;
    vector<float> distance_lut;
    vector<vector<Eigen::Vector3f>> sections;
    vector<BezierSpline> splines;
    ros::NodeHandle nh;
    ros::Rate rate;
    ros::Publisher pub_visualisation;
    int seq_marker;
};
}  // namespace mdi
#endif  // _MDI_TRAJECTORY_HPP_