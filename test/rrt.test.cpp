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

#include "multi_drone_inspection/rrt/rrt.hpp"
#include "multi_drone_inspection/rrt/rrt_builder.hpp"
#include "multi_drone_inspection/utils/rviz/rviz.hpp"
#include "ros/assert.h"

using vec3 = Eigen::Vector3f;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "rtt_test");
    auto nh = ros::NodeHandle();
    auto pub_visualize_rrt = [&nh]() {
        const auto topic_name = "/visualisation_marker";
        return nh.advertise<visualization_msgs::Marker>(topic_name, 10);
    }();

    auto publish_rate = ros::Rate(10);

    auto publish = [&pub_visualize_rrt, &publish_rate](const auto& msg) {
        pub_visualize_rrt.publish(msg);
        publish_rate.sleep();
        ros::spinOnce();
    };

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.01f)
                             .arrow_length(0.1f)
                             .arrow_width(0.02f)
                             .color({0, 1, 0, 1})
                             .build();
    ROS_INFO("creating rrt");
    auto rrt = mdi::rrt::RRT::from_rosparam("/rrt");
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

    std::size_t i = 0;
    rrt.register_cb_for_event_on_new_node_created([&](const auto& p1, const auto& p2) {
        std::cout << "iteration: " << i << '\n';
        ++i;
    });

    // rrt.register_cb_for_event_on_new_node_created([&](const vec3& parent, const vec3&
    // new_node) {
    //     auto msg = arrow_msg_gen({parent, new_node});
    //     publish(msg);
    // });

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
        if (! nh.getParam("/rrt/max_dist_goal_tolerance", goal_tolerance)) {
            std::exit(EXIT_FAILURE);
        }
        return goal_tolerance;
    }();

    for (size_t i = 0; i < 5; i++) {
        publish([&]() {
            auto msg = sphere_msg_gen(goal);
            msg.scale.x = goal_tolerance;
            msg.scale.y = goal_tolerance;
            msg.scale.z = goal_tolerance;
            msg.color.b = 0.6f;
            msg.color.g = 0.f;
            msg.color.a = 0.25f;

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

    std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
              << " calling rrt.run()" << '\n';

    // while (true) {
    //     if (rrt.grow1()) break;
    //     // std::this_thread::sleep_for(std::chrono::microseconds(5000));
    //     std::string input;
    //     std::cout << "press any key (press q to quit)\n";
    //     std::cin.clear();
    //     // std::cin.ignore(INT_MAX, '\n');
    //     std::cout << "input: " << input << '\n';
    //     std::getline(std::cin, input);
    //     std::cout << rrt << std::endl;

    //     if (input == "q") {
    //         break;
    //     }
    // }

    // if (const auto opt = rrt.get_waypoints()) {
    //     const auto waypoints = *opt;
    //     arrow_msg_gen.color.r = 1.0f;
    //     arrow_msg_gen.color.g = 0.0f;
    //     int i = 1;
    //     arrow_msg_gen.scale.x = 0.1f;
    //     arrow_msg_gen.scale.y = 0.1f;
    //     arrow_msg_gen.scale.z = 0.1f;

    //     std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //               << " waypoints.size() == " << waypoints.size() << '\n';

    //     while (ros::ok() && i < waypoints.size()) {
    //         auto& p1 = waypoints[i - 1];
    //         auto& p2 = waypoints[i];
    //         auto arrow = arrow_msg_gen({p1, p2});
    //         publish(arrow);
    //         ++i;
    //     }
    //     std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //               << " i = " << i << '\n';
    // }

    // rrt.print_number_of_root_nodes();

    // const auto wp = rrt.w

    if (const auto opt = rrt.run()) {
        const auto path = *opt;
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " "
                  << " found a solution" << '\n';

        // arrow_msg_gen.color.r = 1.0f;
        // arrow_msg_gen.color.g = 0.0f;
        // int i = 1;
        // arrow_msg_gen.scale.x = 0.1f;
        // arrow_msg_gen.scale.y = 0.1f;
        // arrow_msg_gen.scale.z = 0.1f;
        // while (ros::ok() && i < path.size()) {
        // 	auto& p1 = path[i - 1];
        // 	auto& p2 = path[i];
        // 	auto arrow = arrow_msg_gen({p1, p2});
        // 	publish(arrow);
        // 	++i;
        // }
    }

    // auto i = std::size_t{0};
    // rrt.bft([&](const auto& pt) { ++i; });

    // rrt.print_each_node();

    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " bft reached " << i << " nodes, number of nodes is " << rrt.size() << '\n';

    return 0;
}
