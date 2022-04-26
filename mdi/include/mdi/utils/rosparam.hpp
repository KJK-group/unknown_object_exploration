#ifndef _MULTI_DRONE_INSPECTION_ROSPARAM_HPP_
#define _MULTI_DRONE_INSPECTION_ROSPARAM_HPP_

#include <fmt/core.h>
#include <ros/ros.h>

#include <exception>
#include <stdexcept>
#include <string_view>
// #include <variant>

namespace mdi::utils::rosparam {

// using RosParameterT = std::variant<std::string, int, double, bool, std::vector<RosParameterT>>;

template <typename T>
auto get(std::string_view key) -> T {
    auto default_value = T{};
    if (ros::param::get(key, default_value)) {
        return default_value;
    }

    const auto error_msg = fmt::format("key {} does not exist in the parameter server.", key);
    throw std::invalid_argument(error_msg);
}

}  // namespace mdi::utils::rosparam

#endif  // _MULTI_DRONE_INSPECTION_ROSPARAM_HPP_
