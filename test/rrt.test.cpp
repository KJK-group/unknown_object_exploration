#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "multi_drone_inspection/rrt/rrt.hpp"
#include "multi_drone_inspection/rrt/rrt_builder.hpp"
#include "multi_drone_inspection/utils/rviz/rviz.hpp"

using vec3 = Eigen::Vector3f;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "rtt_test");
    auto nh = ros::NodeHandle();
    auto pub_visualize_rrt = [&nh]() {
        const auto topic_name = "visualisation_marker";
        return nh.advertise<visualization_msgs::Marker>(topic_name, 10);
    }();

    auto publish_rate = ros::Rate(10);

    auto publish = [&pub_visualize_rrt, &publish_rate](const auto& msg) {
        pub_visualize_rrt.publish(msg);
        publish_rate.sleep();
        ros::spinOnce();
        ROS_INFO("publishing marker");
    };

    const auto start = vec3{0, 0, 0};
    const auto goal = vec3{8, 5, 4};

    auto text_msg_gen = mdi::utils::rviz::text_msg_gen{2.0};
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

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.1f)
                             .arrow_length(0.2f)
                             .arrow_width(0.1f)
                             .color({0, 1, 0, 1})
                             .build();

    ROS_INFO("creating rrt");
    auto rrt = mdi::rrt::RRT::builder()
                   .start_and_goal_position(start, goal)
                   .max_iterations(1000)
                   .goal_bias(0.1)
                   .max_dist_goal_tolerance(1.f)
                   .step_size(0.5)
                   .on_new_node_created(
                       [&arrow_msg_gen, &publish](const vec3& parent, const vec3& new_node) {
                           ROS_INFO_STREAM("new_node at " << new_node);
                           auto msg = arrow_msg_gen({parent, new_node});
                           publish(msg);
                       })
                   .on_goal_reached([](const vec3& goal, std::size_t iterations) {
                       std::cout << "found goal " << goal << " in " << iterations << std::endl;
                   })
                   .build();

    std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " "
              << " calling rrt.run()" << std::endl;

    if (const auto opt = rrt.run()) {
        const auto path = *opt;
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " "
                  << " found a solution" << std::endl;

        arrow_msg_gen.color.r = 1.0f;
        arrow_msg_gen.color.g = 0.0f;
        int i = 1;
        while (ros::ok() && i < path.size() - 1) {
            auto& p1 = path[i - 1];
            auto& p2 = path[i];
            auto arrow = arrow_msg_gen({p1, p2});
            publish(arrow);
            ++i;
        }
    }

    return 0;
}
