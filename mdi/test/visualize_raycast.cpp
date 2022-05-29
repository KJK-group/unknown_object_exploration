#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// #include "mdi/rrt/rrt.hpp"
// #include "mdi/rrt/rrt_builder.hpp"
// #include "mdi/octomap.hpp"
#include "mdi/utils/rviz.hpp"
#include "ros/assert.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/spinner.h"

using vec3 = Eigen::Vector3f;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "visualize_raycast");
    auto nh = ros::NodeHandle();
    ros::Duration(2).sleep();
    auto publish_rate = ros::Rate(10);

    auto pub_visualize_rrt = [&nh] {
        const auto topic_name = "/visualization_marker";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::Marker>(topic_name, queue_size);
    }();

    auto publish = [&pub_visualize_rrt, &publish_rate](const auto& msg) {
        for (int i = 0; i < 5; ++i) {
            pub_visualize_rrt.publish(msg);
            publish_rate.sleep();
            ros::spinOnce();
        }
    };

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.15f)
                             .arrow_length(0.3f)
                             .arrow_width(0.05f)
                             .color({0, 1, 0, 1})
                             .build();

    auto sphere_msg_gen = mdi::utils::rviz::sphere_msg_gen{};
    sphere_msg_gen.color = {1, 0, 0, 1};

    // auto octomap_ = mdi::Octomap();

    const auto raycast_vis = [&](const vec3& from, const vec3& to, float x, float y, float z) {
        using std::cos, std::sin, std::acos;
        using mat3x3 = Eigen::Matrix3f;
        using vec3 = Eigen::Vector3f;

        mat3x3 Rx90;
        Rx90 << 1.0f, 0.0f, 0.0f, 0.0f, cos(M_PI_2), sin(M_PI_2), 0.0f, -sin(M_PI_2), cos(M_PI_2);

        const vec3 direction = (to - from);
        const vec3 i_basis = direction.normalized();
        // ensure j_basis orthonormal to i_basis by rotating 90 degrees around
        const vec3 j_basis = Rx90 * i_basis;
        // use cross product to find third orthogonal basis vector.
        const vec3 k_basis = i_basis.cross(j_basis).normalized();
        const auto fmt_vec3 = [](const vec3& v) {
            return "[" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " +
                   std::to_string(v.z()) + "]";
        };

        std::cout << "i_basis " << fmt_vec3(i_basis) << '\n';
        std::cout << "j_basis " << fmt_vec3(j_basis) << '\n';
        std::cout << "k_basis " << fmt_vec3(k_basis) << '\n';

        // T forms a orthonormal basis where i_basis is the direction of from -> to, and j_basis and
        // k_basis span the plane to which i_basis is a normal vector.
        mat3x3 T;
        T.col(0) = i_basis;
        T.col(1) = j_basis;
        T.col(2) = k_basis;

        publish(sphere_msg_gen(to));
        publish(sphere_msg_gen(from));

        const float raycast_length = direction.norm() + 2 * x;

        const auto raycast = [&](const vec3& v) {
            // static constexpr auto convert_to_pt = [](const auto& v) -> mdi::Octomap::point_type {
            // return {v.x(), v.y(), v.z()};
            // };

            const vec3 origin = T * v + from;
            const vec3 target =
                origin + static_cast<vec3>((direction.normalized() * raycast_length));
            std::cout << "origin: " << fmt_vec3(origin) << " -> " << fmt_vec3(target) << '\n';

            publish(arrow_msg_gen({origin, target}));
            // const auto opt = octomap_->raycast_in_direction(origin, direction, raycast_length);

            // if opt is the some variant, then it means that a occupied voxel was hit.
            // return opt.has_value();
            return false;
        };

        // use short circuit evaluation && to lazy evaluate raycasts, so no unnecessary raycasts
        // are performed when it is not needed.
        //     /        /        /
        //    /        /        /
        //   /        /        /
        //  /        /        /
        // 5--------1--------6
        // |   /    |   /    |   /
        // |  /     |  /     |  /
        // | /      | /      | /
        // |/       |/       |/
        // 3--------0--------4
        // |   /    |   /    |   /
        // |  /     |  /     |  /
        // | /      | /      | /
        // |/       |/       |/
        // 7--------2--------8
        return raycast(vec3{0, 0, 0})                // 0, center
               || raycast(vec3{0, y / 2, 0})         // 3, left
               || raycast(vec3{0, -y / 2, 0})        // 4, right
               || raycast(vec3{0, 0, z / 2})         // 1, up
               || raycast(vec3{0, 0, -z / 2})        // 2, down
               || raycast(vec3{0, -y / 2, z / 2})    // 5, up left
               || raycast(vec3{0, y / 2, z / 2})     // 6, up right
               || raycast(vec3{0, y / 2, -z / 2})    // 7, down left
               || raycast(vec3{0, -y / 2, -z / 2});  // 8, down right
    };
    float r = 1;
    const auto from = vec3{0, 0, 0};
    const auto to = vec3{0, 0, 2};
    raycast_vis(from, to, r, r, r * 2);

    ros::spin();

    return 0;
}
