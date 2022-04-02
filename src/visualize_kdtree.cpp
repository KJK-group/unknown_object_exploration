#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <standard_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <string_view>

#include "multi_drone_inspection/utils/random.hpp"

auto create_visualization_msgs_marker(geometry_msgs::Pose pose,
                                      standard_msgs::ColorRGBA std::string_view ns = "kdtree",
                                      std::string_view frame_id = "/map")
    -> visualization_msgs::Marker {
    static unsigned long long id = 0;
    auto msg = visualization_msgs::Marker{};
    msg.ns = ns;
    msg.header.frame_id = frame_id;
    msg.id = id;
    ++id;
    msg.header.stamp = ros::Time::now();

    msg.pose = pose;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.action = visualization_msgs::Marker::ADD;
    msg.color.g = 1.0f;
    msg.color.a = 1.0f;

    return msg;
}

auto format_vector3(const Eigen::Vector3f& vector) -> std::string {
    return "[ " + std::to_string(vector.x) + "," + std::to_string(vector.y) + "," +
           std::to_string(vector.z) + " ]";
}

int main(int argc, char const* argv[]) {
    ros::init(argc, argv, "visualize_kdtree");

    const auto nh = ros::NodeHandle();

    const auto pub_sample_points =
        nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    const auto loop_rate = ros::Rate(10);

    while (ros::ok()) {
        auto pose = geometry_msgs::Pose{};
        pose.orientation.w = 1.0f;

        auto pos = mdi::utils::random::sample_random_point_inside_unit_sphere() * 10.0f;
        pose.position = pos;

        const auto marker_point = create_visualization_msgs_marker(pose);

        ROS_INFO(format_vector3(pos));

        pub_sample_points.publish(marker_point);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
