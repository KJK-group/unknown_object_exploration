#ifndef _MULTI_DRONE_INSPECTION_TF2_HPP_
#define _MULTI_DRONE_INSPECTION_TF2_HPP_

#include <tf2/LinearMath/Transform.h>

#include <eigen3/Eigen/Dense>

#include "geometry_msgs/PointStamped.h"
#include "mdi/utils/utils.hpp"
#include "tf2_ros/transform_listener.h"

namespace mdi::utils::transform {

// loop_rate.sleep();
auto lookup_transform(const tf2_ros::TransformListener& listener) -> tf2_ros::TransformListener {}

auto make_transform(Eigen::Vector3f translation, Eigen::Vector3f euler_rotation) -> geometry_msgs::Transform {
    tf2::Quaternion quaternion;
    quaternion.setRPY(euler_rotation.x(), euler_rotation.y(), euler_rotation.z());

    geometry_msgs::Transform transform;
    transform.rotation.x = quaternion.x();
    transform.rotation.y = quaternion.y();
    transform.rotation.z = quaternion.z();
    transform.rotation.w = quaternion.w();
    transform.translation.x = translation.x();
    transform.translation.y = translation.y();
    transform.translation.z = translation.z();
    return transform;
}

auto transform_vec3(Eigen::Vector3f point, geometry_msgs::TransformStamped transform) -> Eigen::Vector3f {
    geometry_msgs::PointStamped point_in;
    point_in.header.frame_id = transform.header.frame_id;
    point_in.point.x = point.x();
    point_in.point.y = point.y();
    point_in.point.z = point.z();

    geometry_msgs::PointStamped point_out;
    point_out.header.frame_id = transform.child_frame_id;

    tf2::doTransform(point_in, point_out, transform);
    return {point_out.point.x, point_out.point.y, point_out.point.z};
}

auto convert_ned_enu(Eigen::Vector3f point) -> Eigen::Vector3f { return {point.y(), point.x(), -point.z()}; }

auto geometry_mgs_point_to_vec(geometry_msgs::Point point) -> Eigen::Vector3f { return {point.x, point.y, point.z}; }
auto vec_to_geometry_msg_point(Eigen::Vector3f point) -> geometry_msgs::Point {
    geometry_msgs::Point point_out;
    point_out.x = point.x();
    point_out.y = point.y();
    point_out.z = point.z();
    return point_out;
}
// try {
//   listener.waitForTransform("camera", "base_link", ros::Time::now(),
//                             ros::Duration(3.0));
//   listener.lookupTransform("camera", "base_link", ros::Time(0),
//                            stamped_transform);
// } catch (tf::TransformException &ex) {
//   ROS_WARN("%s", ex.what());
// }
}  // namespace mdi::utils::transform

#endif  // _MULTI_DRONE_INSPECTION_TF2_HPP_
