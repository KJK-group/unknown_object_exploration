#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

namespace mdi::trajectory {
class LinearTrajectory {
   public:
    LinearTrajectory(Eigen::Vector3f start, Eigen::Vector3f end);
    auto get_point_at_distance(float distance) -> Eigen::Vector3f;
    auto get_length() -> float;
    auto get_closest_point(Eigen::Vector3f point) -> Eigen::Vector3f;

   private:
    Eigen::Vector3f start;
    Eigen::Vector3f end;
    float length;
};
}  // namespace mdi::trajectory
