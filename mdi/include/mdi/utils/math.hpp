#ifndef _MULTI_DRONE_INSPECTION_MATH_HPP_
#define _MULTI_DRONE_INSPECTION_MATH_HPP_

#include <tf2_eigen/tf2_eigen.h>

#include <cstdint>
#include <functional>
#include <iostream>

namespace mdi::utils {
using Eigen::Vector3f;
using std::uint64_t;

//--------------------------------------------------------------------------------------------------
// Return the factorial of `i`
auto factorial(unsigned int i) -> uint64_t;

//--------------------------------------------------------------------------------------------------
// Returns the binomial coefficient for `n` choose `i`,
// using the factorial function above
auto binomial_coefficient(int n, int i) -> int;

inline auto square(const float x) { return std::pow(x, 2); };

inline auto squared_distance(const Vector3f& v1, const Vector3f& v2) -> float {
    return square(v1.x() - v2.x()) + square(v1.y() - v2.y()) + square(v1.z() - v2.z());
}

inline auto angle_representation(float theta) -> float {
    if (theta > M_PI) {
        return theta -= 2.0 * M_PI;
    } else if (theta < -M_PI) {
        return theta += 2.0 * M_PI;
    }
}

}  // namespace mdi::utils

#endif  // _MULTI_DRONE_INSPECTION_MATH_HPP_
