#ifndef _AMR_HW2_TRANSFORMLISTENER_HPP_
#define _AMR_HW2_TRANSFORMLISTENER_HPP_

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>
#include <optional>

namespace utils::transform {

class TransformListener {
   public:
    TransformListener() = default;
    TransformListener(const TransformListener&) = default;

    auto lookup_tf(const std::string& to_frame, const std::string& from_frame,
                   ros::Time time = ros::Time(0)) -> std::optional<geometry_msgs::TransformStamped>;

    auto transform_vec3(const std::string& to_frame, const std::string& from_frame,
                        const Eigen::Vector3f& vec3, ros::Time time = ros::Time(0))
        -> std::optional<Eigen::Vector3f>;

   private:
    tf2_ros::Buffer buffer = {};
    tf2_ros::TransformListener listener = {buffer};
};
}  // namespace utils::transform

#endif  // _AMR_HW2_TRANSFORMLISTENER_HPP_
