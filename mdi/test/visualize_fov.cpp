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
#include "mdi/bbx.hpp"
#include "mdi/fov.hpp"
#include "mdi/octomap.hpp"
#include "mdi/utils/rviz.hpp"
#include "mdi/voxelstatus.hpp"
#include "octomap/OcTree.h"
#include "octomap/octomap_types.h"
#include "ros/assert.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/spinner.h"

using namespace mdi::types;
using mdi::VoxelStatus;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "visualize_raycast");
    auto nh = ros::NodeHandle();
    auto publish_rate = ros::Rate(50);

    ros::Duration(1).sleep();

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

    auto sphere_msg_gen = mdi::utils::rviz::sphere_msg_gen();
    sphere_msg_gen.scale.x = 0.9;
    sphere_msg_gen.scale.y = 0.9;
    sphere_msg_gen.scale.z = 0.9;

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.15f)
                             .arrow_length(0.3f)
                             .arrow_width(0.05f)
                             .color({1, 0, 0, 1})
                             .build();

    const float resolution = 1;
    auto ocmap = mdi::Octomap(resolution);

    const auto visualize_fov = [&](const Position& pos, const Position& target) {
        // const auto pos = Position{4, 0, 4};
        const auto orientation = Quaternion{1, 0, 0, 0};
        const auto pose = Pose{pos, orientation};
        const auto horizontal = FoVAngle::from_degrees(90);
        const auto vertical = FoVAngle::from_degrees(60);
        const auto depth_range = DepthRange{0.1, 15};

        auto fov = FoV{pose, horizontal, vertical, depth_range, target};
        const auto direction = fov.direction();

        publish_marker(arrow_msg_gen({pos, target}));

        {
            auto msg = arrow_msg_gen({pos, pos + fov.direction() * 5});
            msg.color.r = 0;
            msg.color.b = 1;

            publish_marker(msg);
        }

        const auto bdv = fov.bounding_direction_vectors();
        const auto plane_border = [&](int i, int j) {
            const auto from = bdv[i];
            const auto to = bdv[j];
            // near plane
            {
                auto msg = arrow_msg_gen({fov.pose().position + from * fov.depth_range().min,
                                          fov.pose().position + to * fov.depth_range().min});

                msg.color.r = 0.8;
                msg.color.b = 0.7;
                publish_marker(msg);
            }

            // border
            {
                const auto fmt_vec3 = [&](const vec3& v) -> std::string {
                    return "[" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " +
                           std::to_string(v.z()) + "]";
                };
                const vec3 near = fov.pose().position + from * fov.depth_range().min;
                const vec3 far = fov.pose().position + from * fov.depth_range().max;
                std::cout << "near:" << '\n';
                // std::cout << mdi::types::yaml(near) << '\n';
                std::cout << fmt_vec3(near) << '\n';
                std::cout << "far:" << '\n';
                // std::cout << mdi::types::yaml(far) << '\n';
                std::cout << fmt_vec3(far) << '\n';
                auto msg = arrow_msg_gen({near, far});
                msg.color.r = 0.8;
                msg.color.g = 0.4;
                msg.color.b = 0;
                publish_marker(msg);
            }
            // far plane
            {
                auto msg = arrow_msg_gen({fov.pose().position + from * fov.depth_range().max,
                                          fov.pose().position + to * fov.depth_range().max});
                msg.color.g = 0.8;
                msg.color.b = 0.2;
                publish_marker(msg);
            }
        };

        plane_border(0, 1);
        plane_border(1, 2);
        plane_border(2, 3);
        plane_border(3, 0);

        const auto visualize_bbx = [&](const BBX& bbx) {
            for (auto const& [from, to] : bbx.bounding_edges()) {
                auto msg = arrow_msg_gen({from, to});
                msg.color.r = 0;
                msg.color.g = 1;
                msg.color.b = 1;
                publish_marker(msg);
            }

            {
                auto msg = sphere_msg_gen(bbx.min());
                publish_marker(msg);
            }

            {
                auto msg = sphere_msg_gen(bbx.max());
                msg.color.r = 1;
                msg.color.g = 0;
                publish_marker(msg);
            }
        };

        const auto bbx = mdi::compute_bbx(fov);
        visualize_bbx(bbx);

        for (const auto n : fov.plane_normals()) {
            auto msg = arrow_msg_gen(
                {fov.pose().position, fov.pose().position + n * fov.depth_range().max / 2});
            publish_marker(msg);
        }

        const auto visualize_voxels_inside_fov = [&](const BBX& bbx) {
            // const auto min = bbx.min();
            // const auto max = bbx.max();
            auto cube_msg_gen = mdi::utils::rviz::cube_msg_gen{resolution};
            cube_msg_gen.color = {0, 0, 0, 0.4};

            const auto origin = [&] {
                const auto pos = fov.pose().position;
                return octomap::point3d{pos.x(), pos.y(), pos.z()};
            }();

            const auto visible = [&](const vec3& v) {
                auto opt = ocmap.raycast(origin, octomap::point3d{v.x(), v.y(), v.z()});
                return ! opt.has_value();
            };
            //
            // double gain = 0;
            //
            // double weight_free = 0;
            // double weight_unknown = 0;
            // double weight_occupied = 0;
            //
            ocmap.iterate_over_bbx(bbx, [&](const auto& pt, VoxelStatus vs) {
                const vec3 v = vec3{pt.x(), pt.y(), pt.z()};
                if (fov.inside_fov(v) && visible(v)) {
                    auto msg = cube_msg_gen(v);
                    switch (vs) {
                        case VoxelStatus::Free:
                            msg.color.g = 1;
                            break;

                        case VoxelStatus::Occupied:
                            msg.color.r = 1;
                            break;

                        case VoxelStatus::Unknown:
                            msg.color.b = 1;
                            break;
                    }
                    // msg.scale.x = 0.5;
                    // msg.scale.y = 0.5;
                    // msg.scale.z = 0.5;
                    // msg.color.a = 0.6;
                    publish_marker(msg);
                    // switch (vs) {
                    // case VoxelStatus::Free:
                    // gain += weight_free;
                    // break;
                    // case VoxelStatus::Occupied:
                    // gain += weight_occupied;
                    // break;
                    //
                    // case VoxelStatus::Unknown:
                    // gain += unknown_weight;
                    // break;
                    // }
                }
            });

            // Scale with volume
            // gain *= std::pow(resolution, 3.0);

            // std::cout << "GAIN: " << gain << '\n';
        };

        visualize_voxels_inside_fov(bbx);

        // const float resolution = 0.8f;
        // auto cube_msg_gen = mdi::utils::rviz::cube_msg_gen{resolution};
        // cube_msg_gen.color = {1, 0, 1, 0.4};

        // auto marker_array = visualization_msgs::MarkerArray{};
        // fov.bounding_trapezoid_iter(resolution, [&](float x, float y, float z) {
        //     // std::cout << "[ " << x << ", " << y << ", " << z << " ]" << '\n';
        //     marker_array.markers.push_back(cube_msg_gen({x, y, z}));
        // });

        // publish_marker_array(marker_array);

        // std::cout << yaml(fov) << '\n';
    };

    const auto center = vec3{-10, 10, -7};
    const auto step_size = M_PI_2 / 0.5;
    const auto radius = 40;
    vec3 fem = vec3{2, 2, 2};
    for (float i = 0.0f; i < M_PI * 2; i += step_size) {
        float x = std::cos(i) * radius;
        float y = std::sin(i) * radius;
        const auto offset = vec3{x, y, 0};

        visualize_fov(center + offset, center + fem);
    }

    // for (float i = 0.0f; i < M_PI * 2; i += step_size) {
    //     float x = std::cos(i) * radius;
    //     float z = std::sin(i) * radius;
    //     const auto offset = vec3{x, 0, z};

    //     visualize_fov(center + offset, center + fem);
    // }

    // for (float i = 0.0f; i < M_PI * 2; i += step_size) {
    //     float y = std::cos(i) * radius;
    //     float z = std::sin(i) * radius;
    //     const auto offset = vec3{0, y, z};

    //     visualize_fov(center + offset, center + fem);
    // }

    // for (float i = 0.0f; i < M_PI * 2; i += step_size) {
    //     float x = std::cos(i) * radius;
    //     float y = std::cos(i) * radius;
    //     float z = std::sin(i) * radius;
    //     const auto offset = vec3{x, y, z};

    //     visualize_fov(center + offset, center + fem);
    // }

    publish_rate.sleep();
    ros::spinOnce();

    return 0;
}
