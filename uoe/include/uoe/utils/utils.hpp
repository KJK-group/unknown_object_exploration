#pragma once

#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>
#include <tuple>
#include <vector>

namespace uoe::utils {

constexpr auto DEFAULT_QUEUE_SIZE = 10;
constexpr auto FRAME_WORLD = "world_enu";          // world/global frame
constexpr auto FRAME_BODY = "PX4/odom_local_ned";  // drone body frame
constexpr auto REQUEST_TIMEOUT = 5;
constexpr auto SMALL_DISTANCE_TOLERANCE = 0.1;
constexpr auto DEFAULT_DISTANCE_TOLERANCE = 0.15;
constexpr auto LARGE_DISTANCE_TOLERANCE = 0.2;
constexpr auto DEFAULT_LOOP_RATE = 10;
// escape codes
constexpr auto MAGENTA = "\u001b[35m";
constexpr auto GREEN = "\u001b[32m";
constexpr auto RESET = "\u001b[0m";
constexpr auto BOLD = "\u001b[1m";
constexpr auto ITALIC = "\u001b[3m";
constexpr auto UNDERLINE = "\u001b[4m";

auto hsb_to_rgb(float h, float s, float b) -> std::tuple<float, float, float> {
    assert(h >= 0 && h <= 360);
    assert(s >= 0 && s <= 100);
    assert(b >= 0 && b <= 100);

    // std::cout << "h: " << h << " s: " << s << " b: " << b << std::endl;

    s /= 100;
    b /= 100;

    const auto k = [&](float n) { return std::fmod((n + h / 60), 6); };
    const auto f = [&](float n) {
        return b * (1 - s * std::max(0.0, std::min(std::min(k(n), 4 - k(n)), 1.0)));
    };
    return std::make_tuple(255 * f(5), 255 * f(3), 255 * f(1));
}

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
}  // namespace uoe::utils
