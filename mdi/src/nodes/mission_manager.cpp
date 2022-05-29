#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <vector>

#include "mdi/mission.hpp"

auto main(int argc, char** argv) -> int {
    // ros
    ros::init(argc, argv, "mdi_mission_state");
    auto nh = ros::NodeHandle();
    ros::Rate rate(mdi::utils::DEFAULT_LOOP_RATE);

    // arguments
    auto velocity_target = 0.f;
    auto altitude = 2.f;
    if (argc > 1) velocity_target = std::stof(argv[1]);
    if (argc > 2) altitude = std::stof(argv[2]);

    // mission instance
    auto mission = mdi::Mission(nh, rate, {2.5, 2.5}, velocity_target, {0, 0, altitude}, true);

    // wait for FCU connection
    while (ros::ok() && ! mission.get_drone_state().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mission.run();

    return 0;
}
