#ifndef _MULTI_DRONE_INSPECTION_EIGEN_HPP_
#define _MULTI_DRONE_INSPECTION_EIGEN_HPP_

#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <sstream>
#include <string>
// #include <strstream>

namespace mdi::utils::eigen {

using vec3 = Eigen::Vector3f;

auto format_vector3_as_row_vector(const vec3& v) -> std::string {
    auto ss = std::stringstream{};
    ss << std::setprecision(2) << "[ " << std::to_string(v.x()) << ", " + std::to_string(v.y()) << ", "
       << std::to_string(v.z()) << " ]";
    return ss.str();
}

auto clamp_vec3(Eigen::Vector3f v, float lower_bound, float upper_bound) -> Eigen::Vector3f {
    auto norm = v.norm();
    auto clamped_norm = std::clamp(norm, lower_bound, upper_bound);
    // auto clamped_norm = norm < lower_bound ? lower_bound : norm > upper_bound ? upper_bound : norm;

    return v.normalized() * clamped_norm;
}

}  // namespace mdi::utils::eigen

#endif  // _MULTI_DRONE_INSPECTION_EIGEN_HPP_
