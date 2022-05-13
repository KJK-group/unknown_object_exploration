#ifndef _MDI_LINEAR_TRAJECTORY_HPP_
#define _MDI_LINEAR_TRAJECTORY_HPP_

#include <eigen3/Eigen/Dense>
#include <vector>

namespace mdi::trajectory {
class LinearTrajectory {
   public:
    LinearTrajectory(Eigen::Vector3f start, Eigen::Vector3f end);
    auto get_point_at_distance(float distance) -> Eigen::Vector3f;
    auto get_length() -> float;

   private:
    Eigen::Vector3f start;
    Eigen::Vector3f end;
    float length;
};
}  // namespace mdi::trajectory

#endif  // _MDI_LINEAR_TRAJECTORY_HPP_