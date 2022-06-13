#include <uoe_msgs/NBV.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <stack>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "uoe/utils/rviz.hpp"
#include "uoe_msgs/NBVRequest.h"
#include "uoe_msgs/NBVResponse.h"
#include "ros/forwards.h"
#include "ros/rate.h"
#include "ros/service_client.h"

using vec3 = Eigen::Vector3f;

using namespace std::string_literals;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "rrt_service_test");
    auto nh = ros::NodeHandle();
    auto publish_rate = ros::Rate(10);
    ros::Duration(5).sleep();

    auto client_nbv = [&] {
        const auto service_name = "/uoe/rrt_service/nbv"s;
        return nh.serviceClient<uoe_msgs::NBV>(service_name);
    }();

    auto pub_visualize_path = [&nh] {
        const auto topic_name = "/visualization_marker";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::Marker>(topic_name, queue_size);
    }();

    auto publish = [&](const auto& msg) {
        ROS_INFO("publishing marker");
        for (std::size_t i = 0; i < 3; ++i) {
            pub_visualize_path.publish(msg);
            publish_rate.sleep();
            ros::spinOnce();
        }
    };

    auto request = uoe_msgs::NBVRequest{};
    request.fov.depth_range.min = 0.1;
    request.fov.depth_range.max = 3;
    request.fov.horizontal.angle = 90;
    request.fov.vertical.angle = 60;
    request.fov.pitch.angle = 30;

    request.nbv_config.gain_of_interest_threshold = 83;
    request.nbv_config.weight_free = -1;
    request.nbv_config.weight_occupied = -1;
    request.nbv_config.weight_unknown = 5;
    request.nbv_config.weight_distance_to_object = 5;
    request.nbv_config.voxel_resolution = 0.5;

    request.rrt_config.max_iterations = 1000;
    request.rrt_config.step_size = 2.5;

    const auto start = vec3{0, 0, 0};

    request.rrt_config.start.x = start.x();
    request.rrt_config.start.y = start.y();
    request.rrt_config.start.z = start.z();
    const auto target = vec3{-8, 15, 2.5};

    request.rrt_config.goal.x = target.x();
    request.rrt_config.goal.y = target.y();
    request.rrt_config.goal.z = target.z();

    auto sphere_msg_gen = uoe::utils::rviz::sphere_msg_gen{};
    auto start_msg = sphere_msg_gen(start);
    start_msg.color.r = 0.5;
    start_msg.color.g = 1;
    start_msg.color.b = 0.9;
    start_msg.color.a = 1;
    start_msg.scale.x = 0.2 * 5;
    start_msg.scale.y = 0.2 * 5;
    start_msg.scale.z = 0.2 * 5;
    publish(start_msg);
    // publish goal position
    auto goal_msg = sphere_msg_gen(target);
    goal_msg.color.r = 0;
    goal_msg.color.g = 0;
    goal_msg.color.b = 1;
    goal_msg.color.a = 1;
    goal_msg.scale.x = 0.2 * 5;
    goal_msg.scale.y = 0.2 * 5;
    goal_msg.scale.z = 0.2 * 5;
    publish(goal_msg);

    request.rrt_config.goal_bias = 0.5;
    request.rrt_config.goal_tolerance = 1;
    request.rrt_config.collision_padding = 0.5;
    request.rrt_config.probability_of_testing_full_path_from_new_node_to_goal = 0.0;

    auto nbv_srv = uoe_msgs::NBV{};
    nbv_srv.request = request;

    ROS_INFO("sending nbv request");
    // if (client_nbv.call(nbv_srv)) {
    client_nbv.call(nbv_srv);
    ROS_INFO("got nbv response");

    if (nbv_srv.response.found_nbv_with_sufficent_gain) {
        std::cout << "found gain with sufficient gain" << '\n';
    }
    auto arrow_msg_gen = uoe::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.02f)
                             .arrow_length(0.02f)
                             .arrow_width(0.5f)
                             .color({1, 1, 1, 0.7})
                             .build();
    auto& wps = nbv_srv.response.waypoints;

    auto path = std::vector<vec3>();

    for (const auto& wp : wps) {
        std::cout << wp << std::endl;
        path.emplace_back(wp.x, wp.y, wp.z);
    }

    // visualize result
    // arrow_msg_gen.color.r = 1.0f;
    // arrow_msg_gen.color.g = 0.0f;
    // arrow_msg_gen.color.b = 0.0f;
    size_t i = 1;
    ROS_INFO("visualizing waypoints");
    while (ros::ok() && i < path.size()) {
        auto& p1 = path[i - 1];
        auto& p2 = path[i];
        auto arrow = arrow_msg_gen({p1, p2});
        publish(arrow);
        ++i;
    }

    return 0;
}
