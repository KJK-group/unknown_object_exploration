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
#include "mdi/common_types.hpp"
#include "mdi/utils/rviz/rviz.hpp"
#include "ros/assert.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/spinner.h"
using namespace mdi::types;

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "visualize_raycast");
    auto nh = ros::NodeHandle();
    ros::Duration(3).sleep();
    auto publish_rate = ros::Rate(10);

    auto pub_visualize_marker = [&nh] {
        const auto topic_name = "/visualization_marker";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::Marker>(topic_name, queue_size);
    }();

    auto publish_marker = [&pub_visualize_marker, &publish_rate](const auto& msg) {
        // for (int i = 0; i < 5; ++i) {
        pub_visualize_marker.publish(msg);
        publish_rate.sleep();
        ros::spinOnce();
        // }
    };

    auto pub_visualize_marker_array = [&nh] {
        const auto topic_name = "/visualization_marker_array";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::MarkerArray>(topic_name, queue_size);
    }();

    auto publish_marker_array = [&pub_visualize_marker_array, &publish_rate](const auto& msg) {
        // for (int i = 0; i < 5; ++i) {
        pub_visualize_marker_array.publish(msg);
        publish_rate.sleep();
        ros::spinOnce();
        // }
    };

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.15f)
                             .arrow_length(0.3f)
                             .arrow_width(0.05f)
                             .color({1, 0, 0, 1})
                             .build();

    const auto pos = Position{10, 10, 4};
    const auto orientation = Orientation{0, 0, 0};
    const auto pose = Pose{pos, orientation};
    const auto horizontal = FoVAngle::from_degrees(90);
    const auto vertical = FoVAngle::from_degrees(60);
    const auto depth_range = DepthRange{2, 6};

    auto fov = FoV{pose, horizontal, vertical, depth_range};
    const auto origin = vec3{0, 0, 0};
    const auto direction = fov.direction();

    publish_marker(arrow_msg_gen({origin, direction}));
    arrow_msg_gen.color.r = 0;
    arrow_msg_gen.color.g = 1;

    for (const auto endpoint : fov.compute_endpoints()) {
        publish_marker(arrow_msg_gen({pos, endpoint}));
    }

    const float resolution = 0.8f;
    auto cube_msg_gen = mdi::utils::rviz::cube_msg_gen{resolution};
    cube_msg_gen.color = {1, 0, 1, 0.4};

    auto marker_array = visualization_msgs::MarkerArray{};
    fov.bounding_trapezoid_iter(resolution, [&](float x, float y, float z) {
        // std::cout << "[ " << x << ", " << y << ", " << z << " ]" << '\n';
        marker_array.markers.push_back(cube_msg_gen({x, y, z}));
    });

    publish_marker_array(marker_array);

    std::cout << yaml(fov) << '\n';

    return 0;
}
