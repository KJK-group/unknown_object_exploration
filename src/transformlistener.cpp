#include "utils/transformlistener.hpp"

namespace utils::transform {

auto TransformListener::lookup_tf(const std::string& to_frame, const std::string& from_frame,
                                  ros::Time time)
    -> std::optional<geometry_msgs::TransformStamped> {
    try {
        return buffer.lookupTransform(to_frame, from_frame, time);
    } catch (tf2::TransformException& ex) {
        return std::nullopt;
    }
}

auto TransformListener::transform_vec3(const std::string& to_frame, const std::string& from_frame,
                                       const Eigen::Vector3f& vec3, ros::Time time)
    -> std::optional<Eigen::Vector3f> {
    if (const auto opt = lookup_tf(to_frame, from_frame, time)) {
        const auto tf = *opt;
        auto point_to_frame = geometry_msgs::PointStamped();
        auto point_from_frame = geometry_msgs::PointStamped();
        point_from_frame.point.x = vec3.x();
        point_from_frame.point.y = vec3.y();
        point_from_frame.point.z = vec3.z();
        tf2::doTransform(point_from_frame, point_to_frame, tf);
        const auto& point = point_to_frame.point;
        return Eigen::Vector3f{point.x, point.y, point.z};
    }
    return std::nullopt;
}

}  // namespace utils::transform
