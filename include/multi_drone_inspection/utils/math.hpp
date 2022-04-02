#ifndef _MULTI_DRONE_INSPECTION_MATH_HPP_
#define _MULTI_DRONE_INSPECTION_MATH_HPP_

#include <tf2_eigen/tf2_eigen.h>

using Eigen::Vector3f;

namespace mdi::utils {
//--------------------------------------------------------------------------------------------------
// Return the factorial of `i`
auto factorial(unsigned int i) -> unsigned int;

//--------------------------------------------------------------------------------------------------
// Returns the binomial coefficient for `n` choose `i`,
// using the factorial function above
auto binomial_coefficient(int n, int i) -> int;
}  // namespace mdi::utils

#endif  // _MULTI_DRONE_INSPECTION_MATH_HPP_