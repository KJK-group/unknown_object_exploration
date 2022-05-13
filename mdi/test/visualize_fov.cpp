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
#include <tuple>
#include <utility>
#include <vector>

// #include "mdi/rrt/rrt.hpp"
// #include "mdi/rrt/rrt_builder.hpp"
// #include "mdi/octomap.hpp"
#include "mdi/fov.hpp"
#include "mdi/utils/rviz.hpp"
#include "ros/assert.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/spinner.h"
using namespace mdi::types;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "visualize_raycast");
    auto nh = ros::NodeHandle();
    auto publish_rate = ros::Rate(50);

    auto pub_visualize_marker = [&nh] {
        const auto topic_name = "/visualization_marker";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::Marker>(topic_name, queue_size);
    }();

    auto publish_marker = [&pub_visualize_marker, &publish_rate](const auto& msg) {
        pub_visualize_marker.publish(msg);
        publish_rate.sleep();
        ros::spinOnce();
    };

    auto pub_visualize_marker_array = [&nh] {
        const auto topic_name = "/visualization_marker_array";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::MarkerArray>(topic_name, queue_size);
    }();

    auto publish_marker_array = [&pub_visualize_marker_array, &publish_rate](const auto& msg) {
        pub_visualize_marker_array.publish(msg);
        publish_rate.sleep();
        ros::spinOnce();
    };

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.15f)
                             .arrow_length(0.3f)
                             .arrow_width(0.05f)
                             .color({1, 0, 0, 1})
                             .build();

    const auto visualize_fov = [&](const Position& pos, const Position& target) {
        // const auto pos = Position{4, 0, 4};
        const auto orientation = Quaternion{1, 0, 0, 0};
        const auto pose = Pose{pos, orientation};
        const auto horizontal = FoVAngle::from_degrees(120);
        const auto vertical = FoVAngle::from_degrees(40);
        const auto depth_range = DepthRange{2, 6};

        // const auto target = Position{4, 4, 4};

        auto fov = FoV{pose, horizontal, vertical, depth_range, target};
        const auto origin = vec3{0, 0, 0};
        const auto direction = fov.direction();
        // {
        //     auto msg = arrow_msg_gen({vec3{0, 0, 0}, pos});
        //     msg.color.r = 0.5;
        //     msg.color.g = 0.5;
        //     msg.color.b = 0.5;
        //     publish_marker(msg);
        //     publish_marker(msg);
        //     publish_marker(msg);
        // }

        publish_marker(arrow_msg_gen({pos, target}));

        {
            auto msg = arrow_msg_gen({pos, pos + fov.direction() * 5});
            msg.color.r = 0;
            msg.color.b = 1;

            publish_marker(msg);
        }

        mat3x3 Rx90;
        // BAD
        Rx90 << 1.0f, 0.0f, 0.0f, 0.0f, cos(M_PI_2), sin(M_PI_2), 0.0f, -sin(M_PI_2), cos(M_PI_2);
        mat3x3 Rz90 = mdi::types::rotation_around_Z_axis(M_PI_2);

        auto [i_basis, j_basis, k_basis] = [&] {
            vec3 dir = (target - pos).normalized();
            // 3d plane ax + by + cz + d = 0
            double a = dir.x();
            double b = dir.y();
            double c = dir.z();
            // double d = -pos.dot(dir);
            double d = 0.0;

            // constraint: roll = 0
            // assume: pitch != +- 90 deg
            // 1. find point in plane
            double k = 0;
            auto [i, j] = [&] {
                // 0 - 2pi
                double yaw = std::atan2(dir.y(), dir.x());
                double pi = M_PI;
                if ((0 <= yaw && yaw <= pi / 4) || (3 * pi / 4 <= yaw && yaw <= 5 * pi / 4) ||
                    (7 * pi / 4 <= yaw && yaw <= 2 * pi)) {
                    double j = 10;
                    double i = (-b * j - c * k - d) / a;
                    return std::make_pair(i, j);
                } else {
                    double i = 10;
                    double j = (-a * i - c * k - d) / b;
                    return std::make_pair(i, j);
                }
            }();
            // 2. project to global xy plane
            vec3 foo = vec3{i, j, k}.normalized();
            // 3. point.normalize();

            // 4. find 3rd basis pos.cross(point)
            vec3 bar = dir.cross(foo);

            return std::make_tuple(dir, foo, bar);
        }();

        // const vec3 i_basis = direction_towards_target_;

        // ensure j_basis orthonormal to i_basis by rotating 90 degrees around
        // const vec3 j_basis = Rz90 * i_basis;
        // use cross product to find third orthogonal basis vector.
        // const vec3 k_basis = i_basis.cross(j_basis).normalized();
        // const vec3 k_basis = i_basis.cross(j_basis).normalized();
        // T forms a orthonormal basis where i_basis is the direction of from -> to, and j_basis and k_basis
        // span the plane to which i_basis is a normal vector.
        // T.col(0) = i_basis;
        // T.col(1) = j_basis;
        // T.col(2) = k_basis;

        {
            auto msg = arrow_msg_gen({pos, pos + i_basis});
            msg.color.r = 1;
            publish_marker(msg);
        }

        {
            auto msg = arrow_msg_gen({pos, pos + j_basis});
            msg.color.g = 1;
            publish_marker(msg);
        }

        {
            auto msg = arrow_msg_gen({pos, pos + k_basis});
            msg.color.b = 1;
            publish_marker(msg);
        }

        const auto bdv = fov.bounding_direction_vectors();
        const auto plane_border = [&](int i, int j) {
            const auto from = bdv[i];
            const auto to = bdv[j];
            // near plane
            {
                auto msg = arrow_msg_gen({pos + from * fov.depth_range().min, pos + to * fov.depth_range().min});
                msg.color.r = 0.8;
                msg.color.b = 0.7;
                publish_marker(msg);
            }

            // border
            {
                auto msg = arrow_msg_gen({pos + from * fov.depth_range().min, pos + from * fov.depth_range().max});
                msg.color.r = 0.8;
                msg.color.g = 0.4;
                msg.color.b = 0;
                publish_marker(msg);
            }
            // far plane
            {
                auto msg = arrow_msg_gen({pos + from * fov.depth_range().max, pos + to * fov.depth_range().max});
                msg.color.g = 0.8;
                msg.color.b = 0.2;
                publish_marker(msg);
            }
        };

        plane_border(0, 1);
        plane_border(1, 2);
        plane_border(2, 3);
        plane_border(3, 0);

        // for (const auto d : fov.bounding_direction_vectors()) {
        //     // publish_marker(arrow_msg_gen({pos + d * fov.depth_range().min, pos + d * fov.depth_range().max}));
        //     // ros::Duration(1).sleep();

        //     std::cout << yaml(d) << '\n';
        // }

        // const float resolution = 0.8f;
        // auto cube_msg_gen = mdi::utils::rviz::cube_msg_gen{resolution};
        // cube_msg_gen.color = {1, 0, 1, 0.4};

        // auto marker_array = visualization_msgs::MarkerArray{};
        // fov.bounding_trapezoid_iter(resolution, [&](float x, float y, float z) {
        //     // std::cout << "[ " << x << ", " << y << ", " << z << " ]" << '\n';
        //     marker_array.markers.push_back(cube_msg_gen({x, y, z}));
        // });

        // publish_marker_array(marker_array);

        std::cout << yaml(fov) << '\n';
    };

    const auto center = vec3{-10, 10, -7};
    const auto step_size = M_PI_2 / 4;
    const auto radius = 20;
    vec3 fem = vec3{2, 2, 2};
    for (float i = 0.0f; i < M_PI * 2; i += step_size) {
        float x = std::cos(i) * radius;
        float y = std::sin(i) * radius;
        const auto offset = vec3{x, y, 0};

        visualize_fov(center + offset, center + fem);
    }

    for (float i = 0.0f; i < M_PI * 2; i += step_size) {
        float x = std::cos(i) * radius;
        float z = std::sin(i) * radius;
        const auto offset = vec3{x, 0, z};

        visualize_fov(center + offset, center + fem);
    }

    for (float i = 0.0f; i < M_PI * 2; i += step_size) {
        float y = std::cos(i) * radius;
        float z = std::sin(i) * radius;
        const auto offset = vec3{0, y, z};

        visualize_fov(center + offset, center + fem);
    }

    for (float i = 0.0f; i < M_PI * 2; i += step_size) {
        float x = std::cos(i) * radius;
        float y = std::cos(i) * radius;
        float z = std::sin(i) * radius;
        const auto offset = vec3{x, y, z};

        visualize_fov(center + offset, center + fem);
    }

    publish_rate.sleep();
    ros::spinOnce();

    return 0;
}
