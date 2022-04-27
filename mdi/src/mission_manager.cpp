#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <vector>

#include "mdi/mission.hpp"

auto main(int argc, char** argv) -> int {
    // ros
    ros::init(argc, argv, "mdi_mission_state");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);

    // state
    auto exploration_complete = false;
    auto inspection_complete = false;
    auto first_exploration_iteration = true;

    // Points of interest
    const auto interest_points = std::vector<Eigen::Vector3f>{Eigen::Vector3f(0, 0, 5),   Eigen::Vector3f(10, 10, 7),
                                                              Eigen::Vector3f(18, 9, 15), Eigen::Vector3f(20, 25, 20),
                                                              Eigen::Vector3f(7, 21, 13), Eigen::Vector3f(10, 10, 17),
                                                              Eigen::Vector3f(0, 0, 5)};

    // pass in arguments
    auto velocity_target = 0.f;
    if (argc > 1) velocity_target = std::stof(argv[1]);

    // mission instance
    auto mission = mdi::Mission(&nh, velocity_target);
    for (auto& p : interest_points) {
        mission.add_interest_point(p);
    }

    // wait for FCU connection
    while (ros::ok() && ! mission.get_drone_state().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    auto previous_request_time = ros::Time(0);

    // control loop
    while (ros::ok()) {
        // request to set drone mode to OFFBOARD every 5 seconds until successful
        if (ros::Time::now() - previous_request_time > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
            mission.drone_takeoff();
            previous_request_time = ros::Time::now();
        }
        // request to arm throttle every 5 seconds until succesful
        if (ros::Time::now() - previous_request_time > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
            mission.drone_arm();
            previous_request_time = ros::Time::now();
        }

        switch (mission.state.state) {
            case mdi::Mission::PASSIVE:
                mission.state.state = mdi::Mission::HOME;
                break;
            case mdi::Mission::HOME:
                if (ros::Time::now() - previous_request_time > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
                    auto success = mission.drone_takeoff();
                    previous_request_time = ros::Time::now();
                }
                break;
            case mdi::Mission::EXPLORATION:
                mission.exploration_step();
                break;
            case mdi::Mission::INSPECTION:
                inspection_complete = true;
                mission.state.state = mdi::Mission::HOME;
                break;
            case mdi::Mission::LAND:
                // request to land every 5 seconds until drone is landing
                if (ros::Time::now() - previous_request_time > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
                    mission.drone_land();
                    previous_request_time = ros::Time::now();
                }
                // turn off this node
                break;
            default:
                ROS_WARN_STREAM("Unknown state");
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
