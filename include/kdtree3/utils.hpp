#ifndef _KDTREE3_UTILS_HPP_
#define _KDTREE3_UTILS_HPP_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace kdtree::utils {

/**
 * @brief finds the median of the argument
 * imp ref: https://stackoverflow.com/a/55779158
 * @param numbers
 * @return float
 */
auto median(std::vector<float> numbers) -> float {
    assert(!numbers.empty());
    const auto middle_itr = numbers.begin() + numbers.size() / 2;
    std::nth_element(numbers.begin(), middle_itr, numbers.end());
    if (numbers.size() % 2 == 0) {
        const auto left_middle_itr = std::max_element(numbers.begin(), middle_itr);
        return (*left_middle_itr + *middle_itr) / 2;
    } else {
        return *middle_itr;
    }
}

inline auto square(const float x) { return std::pow(x, 2); };

}  // namespace kdtree::utils

#endif  // _KDTREE3_UTILS_HPP_
