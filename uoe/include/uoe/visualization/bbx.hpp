#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <functional>

#include "uoe/bbx.hpp"
#include "uoe/utils/rviz.hpp"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"

namespace uoe::visualization {

using namespace uoe::types;

auto visualize_bbx(const BBX& bbx, ros::Publisher& publisher) {
    static auto sphere_msg_gen = uoe::utils::rviz::sphere_msg_gen();
    sphere_msg_gen.scale.x = 0.1f;
    sphere_msg_gen.scale.y = 0.1f;
    sphere_msg_gen.scale.z = 0.1f;

    static auto arrow_msg_gen = uoe::utils::rviz::arrow_msg_gen::builder()
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

    for (auto const& [from, to] : bbx.bounding_edges()) {
        auto msg = arrow_msg_gen({from, to});
        msg.color.r = 0;
        msg.color.g = 1;
        msg.color.b = 1;
        publish(msg);
    }

    {
        auto msg = sphere_msg_gen(bbx.min());
        publish(msg);
    }

    {
        auto msg = sphere_msg_gen(bbx.max());
        msg.color.r = 1;
        msg.color.g = 0;
        publish(msg);
    }
};

}  // namespace uoe::visualization
