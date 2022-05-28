#pragma once

namespace mdi::utils::segmentation {
constexpr auto BACKGROUND = 0;
constexpr auto GATE = 0;
constexpr auto TANK = 0;
constexpr auto SEDAN = 0;
constexpr auto TRUCK = 0;
constexpr auto BRIDGE = 0;
constexpr auto PLANE = 0;

/*
BG = margin*0
GATE = margin*1
TANK = margin*2
SEDAN = margin*3
TRUCK = margin*4
BRIDGE = margin*5
PLANE = margin*6
*/
}  // namespace mdi::utils::segmentation