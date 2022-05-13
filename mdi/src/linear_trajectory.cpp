#include "mdi/linear_trajectory.hpp"

#include <iostream>
#include <utility>

namespace mdi::trajectory {
LinearTrajectory::LinearTrajectory(Eigen::Vector3f start, Eigen::Vector3f end)
    : start(std::move(start)), end(std::move(end)) {
    length = (end - start).norm();
}
auto LinearTrajectory::get_length() -> float { return length; }
auto LinearTrajectory::get_point_at_distance(float distance) -> Eigen::Vector3f {
    if (distance < 0) {
        distance = 0;
    } else if (distance > length) {
        distance = length;
    }
    distance = length - distance;
    auto f = distance / length;
    auto diff_vec = end - start;
    return start + f * diff_vec;
}
}  // namespace mdi::trajectory