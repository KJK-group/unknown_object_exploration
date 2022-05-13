
#include <mdi_msgs/RrtFindPath.h>
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

#include "mdi/utils/rviz.hpp"
#include "ros/forwards.h"
#include "ros/rate.h"
#include "ros/service_client.h"

using vec3 = Eigen::Vector3f;

using namespace std::string_literals;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "rrt_service_test");
    auto nh = ros::NodeHandle();
    auto publish_rate = ros::Rate(10);
    ros::Duration(2).sleep();

    auto client_rrt = [&] {
        const auto service_name = "/mdi/rrt_service/find_path"s;
        return nh.serviceClient<mdi_msgs::RrtFindPath>(service_name);
    }();

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

    const auto find_path = [&](const vec3& start, const vec3& end) {
        std::cout << "Finding path from " << start << " to " << end << std::endl;
        const auto goal_tolerance = 2;
        auto rrt_msg = mdi_msgs::RrtFindPath{};
        rrt_msg.request.probability_of_testing_full_path_from_new_node_to_goal = 0;
        rrt_msg.request.goal_bias = 0.7;
        rrt_msg.request.goal_tolerance = goal_tolerance;
        rrt_msg.request.start.x = start.x();
        rrt_msg.request.start.y = start.y();
        rrt_msg.request.start.z = start.z();
        rrt_msg.request.goal.x = end.x();
        rrt_msg.request.goal.y = end.y();
        rrt_msg.request.goal.z = end.z();
        rrt_msg.request.max_iterations = 10000;
        rrt_msg.request.step_size = 2;

        std::vector<Eigen::Vector3f> path;
        if (client_rrt.call(rrt_msg)) {
            auto& waypoints = rrt_msg.response.waypoints;
            // path = std::vector<Eigen::Vector3f>(waypoints.size());
            // std::cout << "before for loop" << std::endl;
            for (auto& wp : waypoints) {
                std::cout << wp << std::endl;
                path.emplace_back(wp.x, wp.y, wp.z);
            }
        }

        auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                                 .arrow_head_width(0.02f)
                                 .arrow_length(0.02f)
                                 .arrow_width(0.02f)
                                 .color({0, 1, 0, 1})
                                 .build();
        // arrow_msg_gen.header.frame_id = utils::FRAME_WORLD;

        auto sphere_msg_gen = mdi::utils::rviz::sphere_msg_gen{};
        // sphere_msg_gen.header.frame_id = utils::FRAME_WORLD;

        // ros::Duration(1).sleep();
        // publish starting position
        auto start_msg = sphere_msg_gen(start);
        start_msg.color.r = 0.5;
        start_msg.color.g = 1;
        start_msg.color.b = 0.9;
        start_msg.color.a = 1;
        start_msg.scale.x = 0.2;
        start_msg.scale.y = 0.2;
        start_msg.scale.z = 0.2;
        publish(start_msg);
        // publish goal position
        auto goal_msg = sphere_msg_gen(end);
        goal_msg.color.r = 0;
        goal_msg.color.g = 1;
        goal_msg.color.b = 0;
        goal_msg.color.a = 1;
        goal_msg.scale.x = 0.2;
        goal_msg.scale.y = 0.2;
        goal_msg.scale.z = 0.2;
        publish(goal_msg);

        // visualise end tolerance
        auto sphere_tolerance_msg = sphere_msg_gen(end);

        auto msg = sphere_msg_gen(end);
        msg.scale.x = goal_tolerance * 2;
        msg.scale.y = goal_tolerance * 2;
        msg.scale.z = goal_tolerance * 2;
        msg.color.r = 1.f;
        msg.color.g = 1.f;
        msg.color.b = 1.f;
        msg.color.a = 0.2f;
        publish(msg);

        // // visualize result
        // arrow_msg_gen.color.r = 0.0f;
        // arrow_msg_gen.color.g = 1.0f;
        // arrow_msg_gen.color.b = 0.0f;
        // arrow_msg_gen.scale.x = 0.05f;
        // arrow_msg_gen.scale.y = 0.05f;
        // arrow_msg_gen.scale.z = 0.05f;
        // int i = 1;
        // while (ros::ok() && i < path.size()) {
        //     auto& p1 = path[i - 1];
        //     auto& p2 = path[i];
        //     auto arrow = arrow_msg_gen({p1, p2});
        //     publish(arrow);
        //     ++i;
        // }
    };

    const vec3 start = {10, 0, 5};
    const vec3 end = {20, 20, 10};

    find_path(start, end);

    std::cout << "DONE" << '\n';

    ros::spin();
    return 0;
}
