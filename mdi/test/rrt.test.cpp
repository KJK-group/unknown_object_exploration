#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

#include "mdi/rrt/rrt.hpp"
#include "mdi/rrt/rrt_builder.hpp"
#include "mdi/utils/rviz/rviz.hpp"
#include "ros/assert.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/spinner.h"

using vec3 = Eigen::Vector3f;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "rtt_test");
    auto nh = ros::NodeHandle();
    ros::Duration(2).sleep();
    auto pub_visualize_rrt = [&nh] {
        const auto topic_name = "/visualization_marker";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::Marker>(topic_name, queue_size);
    }();

    auto pub_marker_array = [&nh] {
        const auto topic_name = "/visualization_marker_array";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::MarkerArray>(topic_name, queue_size);
    }();
    auto publish_rate = ros::Rate(10);

    auto publish_marker_array = [&](const auto& markerarray) {
        pub_marker_array.publish(markerarray);
        publish_rate.sleep();
        ros::spinOnce();
    };

    auto publish = [&pub_visualize_rrt, &publish_rate](const auto& msg) {
        pub_visualize_rrt.publish(msg);
        publish_rate.sleep();
        ros::spinOnce();
    };

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.01f)
                             .arrow_length(0.1f)
                             .arrow_width(0.02f)
                             .color({0, 0.8, 1, 1})
                             .build();
    ROS_INFO("creating rrt");
    auto rrt = mdi::rrt::RRT::from_rosparam("/mdi/rrt");

#ifdef MEASURE_PERF

    if (const auto opt = [=]() -> std::optional<std::filesystem::path> {
            if (argc >= 2) {
                return std::make_optional(std::filesystem::path(argv[1]));
            }
            return std::nullopt;
        }()) {
        const auto measurement_dir = opt.value();
        const auto measurement_file = measurement_dir / "perf.csv";
        rrt.enable_perf_logging(measurement_file);
    }

    // std::size_t i = 0;
    // rrt.register_cb_for_event_on_new_node_created([&](const auto& p1, const auto& p2) {
    //     // std::cout << "iteration: " << i << '\n';
    //     ++i;
    // });

#endif  // MEASURE_PERF

    // rrt.register_cb_for_event_on_trying_full_path(
    //     [](const auto& p1, const auto& p2) { std::cout << "trying full path" << '\n'; });
    // rrt.register_cb_for_event_on_new_node_created([&](const vec3& parent, const vec3& new_node) {
    //     auto msg = arrow_msg_gen({parent, new_node});
    //     publish(msg);
    // });

    // rrt.register_cb_for_event_on_new_node_created(
    // [&](const vec3& parent, const vec3& new_node) { std::cout << "insert new node" << '\n'; });

    const auto start = rrt.start_position();
    const auto goal = rrt.goal_position();

    const auto text_size = 0.8f;
    auto text_msg_gen = mdi::utils::rviz::text_msg_gen{text_size};
    text_msg_gen.color.g = 0.0f;
    text_msg_gen.color.r = 1.0f;
    for (size_t i = 0; i < 5; i++) {
        publish([&]() {
            const auto msg = text_msg_gen("start", start);
            return msg;
        }());
    }

    for (size_t i = 0; i < 5; i++) {
        publish([&]() {
            const auto msg = text_msg_gen("goal", goal);
            return msg;
        }());
    }

		auto sphere_msg_gen = mdi::utils::rviz::sphere_msg_gen{};
    auto sphere_tolerance_msg = sphere_msg_gen(goal);
    const auto goal_tolerance = [&]() {
        float goal_tolerance = 1.0f;
        if (! nh.getParam("/mdi/rrt/max_dist_goal_tolerance", goal_tolerance)) {
            std::exit(EXIT_FAILURE);
        }
        return goal_tolerance;
    }();

    for (size_t i = 0; i < 5; i++) {
        publish([&]() {
            auto msg = sphere_msg_gen(goal);
            msg.scale.x = goal_tolerance * 2;
            msg.scale.y = goal_tolerance * 2;
            msg.scale.z = goal_tolerance * 2;
            msg.color.b = 0.6f;
            msg.color.g = 0.f;
            msg.color.a = 0.1f;

            return msg;
        }());
    }

    // auto rrt = mdi::rrt::RRT::builder()
    //                .start_and_goal_position(start, goal)
    //                .max_iterations([&]() {
    //                    int max_iterations = 500;
    //                    if (!nh.getParam("/rrt/max_iterations", max_iterations)) {
    //                        std::exit(EXIT_FAILURE);
    //                    }
    //                    return max_iterations;
    //                }())
    //                .goal_bias([&]() {
    //                    float goal_bias;
    //                    if (!nh.getParam("/rrt/goal_bias", goal_bias)) {
    //                        std::exit(EXIT_FAILURE);
    //                    }
    //                    return goal_bias;
    //                }())
    //                .probability_of_testing_full_path_from_new_node_to_goal(0.0f)
    //                .max_dist_goal_tolerance(goal_tolerance)
    //                .step_size([&]() {
    //                    int step_size = 1.5;
    //                    if (!nh.getParam("/rrt/step_size", step_size)) {
    //                        std::exit(EXIT_FAILURE);
    //                    }
    //                    return step_size;
    //                }())
    //                .on_new_node_created(
    //                    [&arrow_msg_gen, &publish](const vec3& parent, const vec3& new_node) {
    //                        //    ROS_INFO_STREAM("new_node at " << new_node);
    //                        auto msg = arrow_msg_gen({parent, new_node});
    //                        publish(msg);
    //                    })
    //                .on_goal_reached([](const vec3& goal, std::size_t iterations) {
    //                    std::cout << "found goal " << goal << " in " << iterations << '\n';
    //                })
    //                .on_trying_full_path([](const vec3& new_node, const vec3& goal) {
    //                    std::cout << "trying full path " << '\n';
    //                })
    //                .on_clearing_nodes_in_tree([]() { std::cout << "clearing nodes" << '\n'; })
    //                .build();

    std::cout << rrt << std::endl;

    ROS_INFO_STREAM("calling rrt.run()");

    const auto opt = rrt.run();
    // wait for rviz
    ros::Rate(0.5).sleep();

    {
        auto markerarray = visualization_msgs::MarkerArray{};
        rrt.bft([&](const auto& p1, const auto& p2) {
            auto arrow = arrow_msg_gen({p1, p2});
            markerarray.markers.push_back(arrow);
        });
        publish_marker_array(markerarray);
    }

    if (opt) {
        const auto path = *opt;
        ROS_INFO_STREAM("found solution path");
        arrow_msg_gen.color.r = 0.0f;
        arrow_msg_gen.color.g = 1.0f;
        arrow_msg_gen.color.b = 0.0f;
        int i = 1;
        arrow_msg_gen.scale.x = 0.1f;
        arrow_msg_gen.scale.y = 0.1f;
        arrow_msg_gen.scale.z = 0.1f;

        auto markerarray = visualization_msgs::MarkerArray{};
        for (int i = 1; i < path.size(); ++i) {
            auto& p1 = path[i - 1];
            auto& p2 = path[i];
            auto arrow = arrow_msg_gen({p1, p2});
            markerarray.markers.push_back(arrow);
        }
        publish_marker_array(markerarray);
    }
    std::cout << rrt << std::endl;

    return 0;
}
