#ifndef _MULTI_DRONE_INSPECTION_UTILS_HPP_
#define _MULTI_DRONE_INSPECTION_UTILS_HPP_

#include <cassert>
#include <numeric>
#include <vector>

namespace mdi::utils {

constexpr auto DEFAULT_QUEUE_SIZE = 10;
constexpr auto FRAME_WORLD = "world_enu";          // world/global frame
constexpr auto FRAME_BODY = "PX4/odom_local_ned";  // drone body frame
constexpr auto REQUEST_TIMEOUT = 5;
// escape codes
constexpr auto MAGENTA = "\u001b[35m";
constexpr auto GREEN = "\u001b[32m";
constexpr auto RESET = "\u001b[0m";
constexpr auto BOLD = "\u001b[1m";
constexpr auto ITALIC = "\u001b[3m";
constexpr auto UNDERLINE = "\u001b[4m";

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
