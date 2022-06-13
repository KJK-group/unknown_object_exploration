#pragma once

#include <tf2_eigen/tf2_eigen.h>

#include <cstdint>
#include <functional>
#include <iostream>

namespace uoe::utils {
using Eigen::Vector3f;
using std::uint64_t;

//--------------------------------------------------------------------------------------------------
// Return the factorial of `i`
constexpr auto factorial(unsigned int i) -> uint64_t { return i <= 1 ? 1 : (i * factorial(i - 1)); }

//--------------------------------------------------------------------------------------------------
// Returns the binomial coefficient for `n` choose `i`,
// using the factorial function above
constexpr auto binomial_coefficient(int n, int i) -> int {
    return factorial(n) / (factorial(i) * factorial(n - i));
}

inline auto square(const float x) { return std::pow(x, 2); };

inline auto squared_distance(const Vector3f& v1, const Vector3f& v2) -> float {
    return square(v1.x() - v2.x()) + square(v1.y() - v2.y()) + square(v1.z() - v2.z());
}
}  // namespace uoe::utils
