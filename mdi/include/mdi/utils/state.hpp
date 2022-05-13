#ifndef _MDI_STATE_HPP_
#define _MDI_STATE_HPP_

#include <cmath>

namespace mdi::utils::state {

auto clamp_yaw(float yaw) -> float { return yaw > M_PI ? yaw - 2 * M_PI : yaw < -M_PI ? yaw + 2 * M_PI : yaw; }
}  // namespace mdi::utils::state

#endif  // _MDI_STATE_HPP_