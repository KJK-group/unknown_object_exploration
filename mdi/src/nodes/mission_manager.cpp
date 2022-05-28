#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <vector>

#include "mdi/mission.hpp"

auto main(int argc, char** argv) -> int {
    // ros
    ros::init(argc, argv, "mdi_mission_state");
    auto nh = ros::NodeHandle();
    ros::Rate rate(mdi::utils::DEFAULT_LOOP_RATE);

    // Points of interest
    // const auto interest_points = std::vector<Eigen::Vector3f>{Eigen::Vector3f(10, 10, 7),  Eigen::Vector3f(18, 9,
    // 15),
    //                                                           Eigen::Vector3f(20, 25, 20), Eigen::Vector3f(7, 21,
    //                                                           13), Eigen::Vector3f(10, 10, 17), Eigen::Vector3f(0, 0,
    //                                                           5)};
    // Points of interest
    auto altitude = 2.f;
    const auto interest_points =
        std::vector<Eigen::Vector3f>{Eigen::Vector3f(5, 0, altitude), Eigen::Vector3f(5, 5, altitude + 1),
                                     Eigen::Vector3f(0, 5, altitude), Eigen::Vector3f(0, 0, altitude + 1)};

    // pass in arguments
    auto velocity_target = 0.f;
    if (argc > 1) velocity_target = std::stof(argv[1]);

    // mission instance
    auto mission = mdi::Mission(nh, rate, {2.5, 2.5}, velocity_target, {0, 0, altitude}, true);
    // for (auto& p : interest_points) {
    //     mission.add_interest_point(p);
    // }

    auto idx = 0;
    // mission.add_interest_point(interest_points[idx++]);

    // wait for FCU connection
    while (ros::ok() && ! mission.get_drone_state().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    auto start_time = ros::Time::now();
    ros::Duration delta_time;
    auto point_added = false;

    while (ros::ok()) {
        delta_time = ros::Time::now() - start_time;
        if (delta_time.toSec() > 10 && (idx < interest_points.size())) {
            mission.add_interest_point(interest_points[idx++]);
            start_time = ros::Time::now();
        }
        mission.run_step();
        ros::spinOnce();
        rate.sleep();
    }
    // mission.run();

    return 0;
}
