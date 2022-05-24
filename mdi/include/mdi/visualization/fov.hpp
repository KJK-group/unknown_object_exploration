#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <functional>

#include "mdi/bbx.hpp"
#include "mdi/fov.hpp"
#include "mdi/octomap.hpp"
#include "mdi/utils/rviz.hpp"
#include "mdi/voxelstatus.hpp"
#include "ros/publisher.h"

namespace mdi::visualization {

using namespace mdi::types;

auto visualize_fov(const FoV& fov, ros::Publisher& publisher) {
    static auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                                    .arrow_head_width(0.0f)
                                    .arrow_length(0.02f)
                                    .arrow_width(0.02f)
                                    .color({1, 0, 0, 1})
                                    .build();
    const auto publish = [&](auto msg) {
        publisher.publish(msg);
        ros::spinOnce();
        ros::Rate(10).sleep();
    };

    // publish(arrow_msg_gen({fov.pose().position, fov.target()}));

    {
        auto msg = arrow_msg_gen({fov.pose().position, fov.pose().position + fov.direction() * 5});
        msg.color.r = 0;
        msg.color.b = 1;
        msg.scale.z = 0.2;
        msg.scale.y = 0.2;

        publish(msg);
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
            publish(msg);
        }

        // border
        {
            const vec3 near = fov.pose().position + from * fov.depth_range().min;
            const vec3 far = fov.pose().position + from * fov.depth_range().max;
            auto msg = arrow_msg_gen({near, far});
            msg.color.r = 0.8;
            msg.color.g = 0.4;
            msg.color.b = 0;
            publish(msg);
        }
        // far plane
        {
            auto msg = arrow_msg_gen({fov.pose().position + from * fov.depth_range().max,
                                      fov.pose().position + to * fov.depth_range().max});
            msg.color.g = 0.8;
            msg.color.b = 0.2;
            publish(msg);
        }
    };

    plane_border(0, 1);
    plane_border(1, 2);
    plane_border(2, 3);
    plane_border(3, 0);
}

auto visualize_voxels_inside_fov(const FoV& fov, const mdi::Octomap& ocmap, float resolution,
                                 ros::Publisher& publisher) {
    using mdi::VoxelStatus;

    const auto bbx = mdi::compute_bbx(fov);

    const auto publish = [&](auto msg) {
        publisher.publish(msg);
        ros::spinOnce();
        ros::Rate(10).sleep();
    };

    static auto cube_msg_gen = mdi::utils::rviz::cube_msg_gen{resolution};

    const auto origin = [&] {
        const auto pos = fov.pose().position;
        return octomap::point3d{pos.x(), pos.y(), pos.z()};
    }();

    const auto visible = [&](const vec3& v) {
        auto /* opt */ voxel = ocmap.raycast(origin, octomap::point3d{v.x(), v.y(), v.z()});
        // return ! opt.has_value();

        auto match = Overload{
            [](Free _) { return true; },
            [](Unknown _) { return false; },
            [](Occupied _) { return false; },
        };

        return std::visit(match, voxel);
    };

    auto msgs = visualization_msgs::MarkerArray{};

    ocmap.iterate_over_bbx(bbx, [&](const auto& pt, VoxelStatus vs) {
        const vec3 v = vec3{pt.x(), pt.y(), pt.z()};
        if (fov.inside_fov(v) && visible(v)) {
            auto msg = cube_msg_gen(v);
            // msg.color = mdi::utils::rviz::RGBA{0, 0, 0, 0.4};
            msg.color.r = 0;
            msg.color.g = 0;
            msg.color.b = 0;
            msg.color.a = 0.4;
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
            msgs.markers.push_back(msg);
        }
    });

    publish(msgs);
};

}  // namespace mdi::visualization
