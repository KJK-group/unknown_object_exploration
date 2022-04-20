#ifndef _MULTI_DRONE_INSPECTION_TIME_HPP_
#define _MULTI_DRONE_INSPECTION_TIME_HPP_

#include <chrono>

namespace mdi::utils::time {
// make the decltype slightly easier to the eye
using seconds_t = std::chrono::seconds;

auto get_seconds_since_epoch() -> decltype(seconds_t().count()) {
    const auto now = std::chrono::system_clock::now();
    const auto epoch = now.time_since_epoch();
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    return seconds.count();
}
}  // namespace mdi::utils::time

#endif  // _MULTI_DRONE_INSPECTION_TIME_HPP_
