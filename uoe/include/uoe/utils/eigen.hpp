#pragma once

#include <iomanip>
#include <sstream>
#include <string>

#include "../common_types.hpp"
// #include <strstream>

namespace uoe::utils::eigen {

auto format_vector3_as_row_vector(const types::vec3& v) -> std::string {
    auto ss = std::stringstream{};
    ss << std::setprecision(2) << "[ " << std::to_string(v.x()) << ", " + std::to_string(v.y())
       << ", " << std::to_string(v.z()) << " ]";
    return ss.str();
}

auto clamp_vec3(types::vec3 v, float lower_bound, float upper_bound) -> types::vec3 {
    auto norm = v.norm();
    auto clamped_norm = std::clamp(norm, lower_bound, upper_bound);
    // auto clamped_norm = norm < lower_bound ? lower_bound : norm > upper_bound ? upper_bound :
    // norm;

    return v.normalized() * clamped_norm;
}

auto floor_vec3(types::vec3 v) -> types::vec3 { return {std::floor(v.x()), std::floor(v.y()), std::floor(v.z())}; }

}  // namespace uoe::utils::eigen
