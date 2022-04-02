#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <multi_drone_inspection/voxblox_manager.hpp>

using Eigen::Vector3d;

// #if defined(__GNUC__) || defined(__GNUG__)
// #include <string>
// #define NODE_NAME std::string(__FILE_NAME__).substr(0, std::string(__FILE_NAME__).rfind(".cpp"))
// #else
// #define NODE_NAME ""
// #endif

auto main(int argc, char** argv) -> int {
    // ROS initialisations
    ros::init(argc, argv, "test_voxblox");
    auto nh = ros::NodeHandle();
    auto nh_private = ros::NodeHandle("~");
    ros::Rate rate(20.0);

    auto voxblox_manager = mpi::VoxbloxManager(nh, nh_private);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_STREAM(voxblox_manager.get_map_distance(Vector3d(0.f, 0.f, 1.f)));
        // ROS_INFO(std::to_string(voxblox_manager.get_map_distance(Vector3d(0.f, 0.f, 1.f))));
    }

    return 0;
}
