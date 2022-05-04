#include <ros/ros.h>

#include <vector>

#include "mdi/trajectory.hpp"

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "mdi_trajectory_tester");
    auto nh = ros::NodeHandle();
    auto rate = ros::Rate(mdi::utils::DEFAULT_LOOP_RATE * 2);

    auto pub_visualisation =
        nh.advertise<visualization_msgs::Marker>("/mdi/visualisation/marker", mdi::utils::DEFAULT_QUEUE_SIZE);

    auto marker_scale = 0.1;
    auto path = std::vector<Eigen::Vector3f>{
        {2, 2, 2},      {3, 3, 3},      {4, 4, 4},       {4.5, 6, 3.5},   {3.5, 5, 4.5},  {2.5, 4, 3.5},
        {1.5, 3, 2.5},  {2, 3.5, 4.5},  {3, 3, 6},       {4, 2.5, 7.5},   {5, 2, 9},      {6, 1.5, 10.5},
        {3, 1.5, 10.5}, {0, 1.5, 10.5}, {-3, 1.5, 10.5}, {-3, -2.5, 9.5}, {-3, -2.5, 6.5}};
    auto t = mdi::Trajectory(nh, rate, path, marker_scale);

    std::cout << "total length = " << t.get_length() << std::endl;

    auto start_time = ros::Time::now();
    auto delta_time = ros::Time::now() - start_time;
    auto velocity = 1;

    auto seq_marker = 0;
    visualization_msgs::Marker m;
    m.header.frame_id = mdi::utils::FRAME_WORLD;
    m.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::SPHERE;
    m.color.a = 1;
    m.color.r = 1;
    m.color.g = 1;
    m.color.b = 1;
    m.scale.x = marker_scale;
    m.scale.y = marker_scale;
    m.scale.z = marker_scale;

    while (ros::ok()) {
        delta_time = ros::Time::now() - start_time;

        auto point = t.get_point_at_distance(delta_time.toSec() * velocity);

        m.id = seq_marker;
        m.header.seq = seq_marker++;
        m.pose.position.x = point.x();
        m.pose.position.y = point.y();
        m.pose.position.z = point.z();

        pub_visualisation.publish(m);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}