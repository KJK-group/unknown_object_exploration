#ifndef _MULTI_DRONE_INSPECTION_UTILS_HPP_
#define _MULTI_DRONE_INSPECTION_UTILS_HPP_

#include <cassert>
#include <numeric>
#include <vector>


namespace mdi::utils {

auto range(int end) -> std::vector<int> {
    auto result = std::vector<int>(end);
    std::iota(result.begin(), result.end(), 0);
    return result;
}

auto range(int start, int end) -> std::vector<int> {
    assert(start < end);
    auto size = end - start;
    auto result = std::vector<int>(size);
    std::iota(result.begin(), result.end(), start);

    return result;
}

// auto range(int start, int end, unsigned int step) -> std::vector<int> {
// 	assert(start < end && step > 0);
// 	auto size = end - start;
// 	auto result = std::vector<int>(size);

// 	return result;
// }
}  // namespace mdi::utils

#endif  // _MULTI_DRONE_INSPECTION_UTILS_HPP_
