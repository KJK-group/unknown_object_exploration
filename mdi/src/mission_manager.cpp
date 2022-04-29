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
    ros::Rate rate(mdi::utils::DEFAULT_LOOP_RATE);

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
    auto mission = mdi::Mission(nh, rate, velocity_target);
    // for (auto& p : interest_points) {
    //     mission.add_interest_point(p);
    // }

    auto idx = 0;
    mission.add_interest_point(interest_points[idx++]);

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

    auto start_time = ros::Time::now();
    ros::Duration delta_time;
    auto point_added = false;

    while (ros::ok()) {
        delta_time = ros::Time::now() - start_time;
        if (delta_time.toSec() > 40 && ! point_added) {
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
