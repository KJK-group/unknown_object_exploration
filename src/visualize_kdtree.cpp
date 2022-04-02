#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <string_view>

#include "multi_drone_inspection/utils/random.hpp"

using u8 = unsigned char;

struct rgba {
    float r, g, b, a;
};

struct scale {
    float x = 1.f, y = 1.f, z = 1.f;
};

auto create_visualization_msgs_marker(geometry_msgs::Pose pose, rgba color = {0, 1, 0, 1},
                                      scale scl = {0.1, 0.1, 0.1}, std::string_view ns = "kdtree",
                                      std::string_view frame_id = "map")
    -> visualization_msgs::Marker {
    static unsigned long long id = 0;
    auto msg = visualization_msgs::Marker{};
    msg.ns = ns;
    msg.header.frame_id = frame_id;
    msg.id = id;
    ++id;
    msg.header.stamp = ros::Time::now();
    msg.lifetime = ros::Duration(0);

    msg.pose = pose;
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.color.r = color.r;
    msg.color.g = color.g;
    msg.color.b = color.b;
    msg.color.a = color.a;
    msg.scale.x = scl.x;
    msg.scale.y = scl.y;
    msg.scale.z = scl.z;

    return msg;
}

auto format_vector3(const Eigen::Vector3f& vector) -> std::string {
    return "[ " + std::to_string(vector.x()) + "," + std::to_string(vector.y()) + "," +
           std::to_string(vector.z()) + " ]";
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "visualize_kdtree");

    auto nh = ros::NodeHandle();

    auto pub_sample_points = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    auto loop_rate = ros::Rate(10);

    const auto n = 4;
    auto positions = std::vector<Eigen::Vector3f>();
    for (auto i = 0; i < n; i++) {
        for (auto j = 0; j < n; j++) {
            for (auto k = 0; k < n; k++) {
                positions.emplace_back(i, j, k);
            }
        }
    }

    auto r = 10.f;
    auto bounding_sphere_center = Eigen::Vector3f::Ones() * 5.f;

    auto pose = geometry_msgs::Pose{};
    pose.orientation.w = 1.0f;
    pose.position.x = bounding_sphere_center.x();
    pose.position.y = bounding_sphere_center.y();
    pose.position.z = bounding_sphere_center.z();

    // pose.position.x = 1;
    // pose.position.y = 1;
    // pose.position.z = 1;
    ROS_INFO("publishing bounding sphere");
    auto bounding_sphere =
        create_visualization_msgs_marker(pose, rgba{0, 0, 0.8, 0.2}, scale{r, r, r});
    for (int i = 0; i < 10; i++) {
        pub_sample_points.publish(bounding_sphere);
        ros::spinOnce();
        loop_rate.sleep();
    }

    auto direction = Eigen::Vector3f{1, 0.8, 0.3}.normalized();

    auto count = 0;
    while (ros::ok() and count < 1000) {
        auto pose = geometry_msgs::Pose{};
        pose.orientation.w = 1.0f;
        // const auto& pos = positions.at(count % positions.size());

        auto pos =
            mdi::utils::random::sample_random_point_inside_unit_sphere(direction, 0) * (r / 2.f) +
            bounding_sphere_center;

        std::cout << "position" << pos << std::endl;
        // std::cout << format_vector3(pos) << std::endl;
        pose.position.x = pos.x();
        pose.position.y = pos.y();
        pose.position.z = pos.z();
        // pose.position.x = pos.x;
        // pose.position.y = pos.y;
        // pose.position.z = pos.z;
        const auto marker_point = create_visualization_msgs_marker(pose);

        // ROS_INFO(format_vector3(pos).c_str());

        pub_sample_points.publish(marker_point);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
