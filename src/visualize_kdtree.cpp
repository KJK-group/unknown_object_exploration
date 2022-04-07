#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <string_view>
#include <variant>

#include "kdtree3/kdtree3.hpp"
#include "multi_drone_inspection/utils/random.hpp"
#include "multi_drone_inspection/utils/rviz/rviz.hpp"

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

// using vec3 = std::variant<Eigen::Vector3f, Eigen::Vector3d, geometry_msgs::Point>;

// auto format_vec3(const vec3& v) -> std::string {
// 	switch (v.index()) {
// 		case 0: return
// 		case 1: return
// 		case 2: return
// 	}

// }

auto format_vector3(const Eigen::Vector3f& vector) -> std::string {
    return "[ " + std::to_string(vector.x()) + "," + std::to_string(vector.y()) + "," +
           std::to_string(vector.z()) + " ]";
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "visualize_kdtree");
    auto nh = ros::NodeHandle();
    auto pub_sample_points = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    auto loop_rate = ros::Rate(10);
    auto publish = [&](const visualization_msgs::Marker& marker) {
        pub_sample_points.publish(marker);
        ros::spinOnce();
        loop_rate.sleep();
    };

    // for (int i = 0; i < 10000; i++) {
    //     std::cout << "random " << mdi::utils::random::random01() << std::endl;
    // }

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

    ROS_INFO("publishing bounding sphere");
    auto bounding_sphere =
        create_visualization_msgs_marker(pose, rgba{0, 0, 0.8, 0.2}, scale{r, r, r});
    for (int i = 0; i < 10; i++) {
        publish(bounding_sphere);
    }

    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();
    // mdi::utils::random::sample_random_point_inside_unit_sphere();

    auto direction = Eigen::Vector3f{1, 0.8, 0.3}.normalized();
    auto n_points = 1000;
    auto points = std::vector<Eigen::Vector3f>(n_points);
    for (size_t i = 0; i < n_points; ++i) {
        points[i] = (mdi::utils::random::sample_random_point_inside_unit_sphere(direction, 0) * r +
                     bounding_sphere_center);
    }
    auto kdtree = kdtree::kdtree3{points};

    // auto count = 0;
    // auto sphere_msg = mdi::utils::rviz::sphere_msg_gen{};

    // while (ros::ok() and count < 1000) {
    //     auto pose = geometry_msgs::Pose{};
    //     pose.orientation.w = 1.0f;
    //     // const auto& pos = positions.at(count % positions.size());

    //     auto pos =
    //         mdi::utils::random::sample_random_point_inside_unit_sphere(direction, 0) * (r / 2.f)
    //         + bounding_sphere_center;

    //     std::cout << "position" << pos << std::endl;
    //     // std::cout << format_vector3(pos) << std::endl;
    //     pose.position.x = pos.x();
    //     pose.position.y = pos.y();
    //     pose.position.z = pos.z();
    //     // pose.position.x = pos.x;
    //     // pose.position.y = pos.y;
    //     // pose.position.z = pos.z;
    //     const auto marker_point = sphere_msg(pose);

    //     // ROS_INFO(format_vector3(pos).c_str());

    //     pub_sample_points.publish(marker_point);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     ++count;
    // }

    auto text_msg = mdi::utils::rviz::text_msg_gen{};
    publish(text_msg("start", geometry_msgs::Pose{}));

    auto end_pos = geometry_msgs::Pose{};
    end_pos.position.x = 10;
    end_pos.position.y = 10;
    end_pos.position.z = 10;
    text_msg.color.r = 0.5f;
    publish(text_msg("end", end_pos));

    {
        auto nodes_in_kdtree = std::vector<Eigen::Vector3f>();

        kdtree.postorder_traversal([&](const auto& node) { nodes_in_kdtree.push_back(node); });
        // auto pose = geometry_msgs::Pose{};
        // pose.orientation.w = 1.0f;
        // auto marker_point = create_visualization_msgs_marker(pose);
        // marker_point.type = visualization_msgs::Marker::ARROW;
        // marker_point.color.r = 1.0f;

        // std::for_each(nodes_in_kdtree.begin(), nodes_in_kdtree.end(), [&](const auto& node) {
        //     auto p = geometry_msgs::Point{};
        //     p.x = node.x();
        //     p.y = node.y();
        //     p.z = node.z();
        //     marker_point.points.push_back(p);
        // });

        using mdi::utils::rviz::Arrow;

        auto arrow_msg = mdi::utils::rviz::arrow_msg_gen{};
        arrow_msg.color.r = 1.0f;
        arrow_msg.color.g = 0.0f;
        arrow_msg.color.a = 0.6f;
        arrow_msg.scale.x = 0.05f;
        arrow_msg.scale.y = 0.05f;
        arrow_msg.scale.z = 0.05f;

        int i = 1;
        while (ros::ok() && i < points.size() - 1) {
            auto& v1 = nodes_in_kdtree[i - 1];
            auto& v2 = nodes_in_kdtree[i];
            auto marker_point = arrow_msg({v1, v2});
            publish(marker_point);
            ++i;
        }
    }

    std::cout << "depth of kdtree is " << kdtree.max_depth() << std::endl;

    return 0;
}
