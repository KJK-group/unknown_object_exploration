#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <vector>

#include "mdi/mission.hpp"

mdi_msgs::PointNormStamped error;
auto error_cb(const mdi_msgs::PointNormStamped::ConstPtr& msg) { error = *msg; }

auto main(int argc, char** argv) -> int {
    // ros
    ros::init(argc, argv, "mdi_mission_state");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);

    // state
    auto exploration_complete = false;
    auto inspection_complete = false;
    auto first_exploration_iteration = true;

    auto sub_error = nh.subscribe("/mdi/error", mdi::utils::DEFAULT_QUEUE_SIZE, error_cb);

    // Points of interest
    const auto interest_points = std::vector<Eigen::Vector3f>{Eigen::Vector3f(10, 10, 7),  Eigen::Vector3f(18, 9, 15),
                                                              Eigen::Vector3f(20, 25, 20), Eigen::Vector3f(7, 21, 13),
                                                              Eigen::Vector3f(10, 10, 17), Eigen::Vector3f(0, 0, 5)};

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

    auto previous_request_time_mode = ros::Time(0);
    auto previous_request_time_arm = ros::Time(0);
    auto previous_request_time_takeoff = ros::Time(0);
    auto previous_request_time_land = ros::Time(0);

    auto success = false;
    std::string print;

    // control loop
    while (ros::ok()) {
        // request to set drone mode to OFFBOARD every 5 seconds until successful
        if (ros::Time::now() - previous_request_time_mode > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
            mission.drone_set_mode();
            previous_request_time_mode = ros::Time::now();
        }
        // request to arm throttle every 5 seconds until succesful
        if (ros::Time::now() - previous_request_time_arm > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
            mission.drone_arm();
            previous_request_time_arm = ros::Time::now();
        }

        switch (mission.state.state) {
            case mdi::Mission::PASSIVE:
                mission.state.state = mdi::Mission::HOME;
                break;
            case mdi::Mission::HOME:
                mission.drone_takeoff();
                if (error.norm < 0.2) {
                    mission.state.state = mdi::Mission::EXPLORATION;
                }
                break;
            case mdi::Mission::EXPLORATION:
                if (ros::Time::now() - previous_request_time_mode > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
                    success = mission.drone_set_mode();
                    print = success ? "mode set success" : "mode set failure";
                    std::cout << print << std::endl;
                    previous_request_time_mode = ros::Time::now();
                }
                if (mission.get_drone_state().mode == "OFFBOARD") {
                    mission.exploration_step();
                }
                break;
            case mdi::Mission::INSPECTION:
                inspection_complete = true;
                mission.state.state = mdi::Mission::HOME;
                break;
            case mdi::Mission::LAND:
                // request to land every 5 seconds until drone is landing
                if (ros::Time::now() - previous_request_time_land > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
                    success = mission.drone_land();
                    print = success ? "land success" : "land failure";
                    std::cout << print << std::endl;
                    previous_request_time_land = ros::Time::now();
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