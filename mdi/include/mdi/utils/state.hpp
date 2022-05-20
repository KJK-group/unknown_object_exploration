#pragma once

#include <cmath>

namespace mdi::utils::state {

constexpr auto yaw_representation(float angle) -> float {
    return angle > M_PI ? angle - 2 * M_PI : angle < -M_PI ? angle + 2 * M_PI : angle;
}
}  // namespace mdi::utils::state
