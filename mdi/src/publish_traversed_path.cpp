#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "mdi/utils/utils.hpp"
#include "mdi_msgs/MissionStateStamped.h"

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "publish_traversed_path");
    auto nh = ros::NodeHandle();
    auto spin_rate = ros::Rate(mdi::utils::DEFAULT_LOOP_RATE);

    auto path_est = nav_msgs::Path{};
    auto path_target = nav_msgs::Path{};
    path_est.header.frame_id = mdi::utils::FRAME_WORLD;
    path_target.header.frame_id = mdi::utils::FRAME_WORLD;

    auto sub_odom = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10,
        [&path_est](const geometry_msgs::PoseStamped::ConstPtr& msg) { path_est.poses.push_back(*msg); });
    auto sub_target = nh.subscribe<mdi_msgs::MissionStateStamped>(
        "/mdi/state", 10, [&path_target](const mdi_msgs::MissionStateStamped::ConstPtr& msg) {
            geometry_msgs::PoseStamped pose;
            pose.header = msg->header;
            pose.pose = msg->target;
            path_target.poses.push_back(pose);
        });

    auto pub_path_est = nh.advertise<nav_msgs::Path>("/mdi/paths/estimated", 10);
    auto pub_path_target = nh.advertise<nav_msgs::Path>("/mdi/paths/target", 10);
    auto seq_est = 0;
    auto seq_target = 0;

    auto publish = [&](auto& msg, ros::Publisher& publisher, int seq) {
        msg.header.seq = seq;
        msg.header.stamp = ros::Time::now();
        publisher.publish(msg);
        ROS_INFO("published path");
        ros::spinOnce();
        spin_rate.sleep();
        // path_est.poses.clear();
    };

    auto publish_rate = ros::Rate(5);
    while (ros::ok()) {
        publish_rate.sleep();
        publish(path_est, pub_path_est, seq_est++);
        publish(path_target, pub_path_target, seq_target++);
    }

    return 0;
}
