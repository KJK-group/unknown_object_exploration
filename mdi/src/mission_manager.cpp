#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "boost/format.hpp"
#include "mdi/bezier_spline.hpp"
#include "mdi/mission.hpp"
#include "mdi/rrt/rrt.hpp"
#include "mdi/rrt/rrt_builder.hpp"
#include "mdi/utils/rviz/rviz.hpp"
#include "mdi/utils/transformlistener.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h"

constexpr auto TOLERANCE_DISTANCE = 0.1;
constexpr auto TARGET_VELOCITY = 1.f;              // move drone at TARGET_VELOCITY m/s
constexpr auto FRAME_WORLD = "world_enu";          // world/global frame
constexpr auto FRAME_BODY = "PX4/odom_local_ned";  // drone body frame
constexpr auto SPLINE_Z_OFFSET = 5;

//--------------------------------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------------------------------

auto main(int argc, char** argv) -> int {
    //----------------------------------------------------------------------------------------------
    // ROS initialisations
    ros::init(argc, argv, "mdi_mission_state");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    // save start_time

    // state
    auto exploration_complete = false;
    auto inspection_complete = false;
    auto first_exploration_iteration = true;

    // Points of interest
    const auto interest_points = std::vector<Eigen::Vector3f>{Eigen::Vector3f(0, 0, 5),   Eigen::Vector3f(10, 10, 7),
                                                              Eigen::Vector3f(18, 9, 15), Eigen::Vector3f(20, 25, 20),
                                                              Eigen::Vector3f(7, 21, 13), Eigen::Vector3f(10, 10, 17),
                                                              Eigen::Vector3f(0, 0, 5)};

    //----------------------------------------------------------------------------------------------
    // pass in arguments
    if (argc > 1) velocity_target = std::stof(argv[1]);

    auto mission = mdi::Mission(&nh, velocity_target);
    for (auto& p : interest_points) {
        mission.add_interest_point(p);
    }

    //----------------------------------------------------------------------------------------------
    // wait for FCU connection
    while (ros::ok() && ! mission.get_drone_state().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    auto previous_request_time = ros::Time(0);

    //----------------------------------------------------------------------------------------------
    // control loop
    while (ros::ok()) {
        //------------------------------------------------------------------------------------------
        // request to set drone mode to OFFBOARD every 5 seconds until successful
        if (ros::Time::now() - previous_request_time > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
            mission.drone_takeoff();
            previous_request_time = ros::Time::now();
        }
        //------------------------------------------------------------------------------------------
        // request to arm throttle every 5 seconds until succesful
        if (ros::Time::now() - previous_request_time > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
            mission.drone_arm();
            previous_request_time = ros::Time::now();
        }

        auto delta_time = ros::Time::now() - start_time;
        float distance;
        float remaining_distance;
        Eigen::Vector3f start;
        Eigen::Vector3f end;
        std::vector<Eigen::Vector3f> path;

        switch (mission.state.state) {
            case PASSIVE:
                mission.state.state = HOME;
                break;
            case HOME:
                // 1. if within tolerance, go to EXPLORATION
                // 2. if it's the second time we're here, go to LAND

                //------------------------------------------------------------------------------------------
                // request to arm throttle every 5 seconds until succesful
                if (ros::Time::now() - previous_request_time > ros::Duration(mdi::utils::REQUEST_TIMEOUT)) {
                    auto success = mission.takeoff();
                    previous_request_time = ros::Time::now();
                }
                // std::cout << MAGENTA << error << RESET << std::endl;
                // if (error.norm < TOLERANCE_DISTANCE && delta_time.toSec() > 5) {
                //     if (inspection_complete) {
                //         mission.state.state = LAND;
                //     } else {
                //         // start_time = ros::Time::now();
                //         first_exploration_iteration = true;
                //         mission.state.state = EXPLORATION;
                //     }
                // }
                break;
            case EXPLORATION:
                if (end_idx > interest_points.size()) {
                    exploration_complete = true;
                    mission.state.state = INSPECTION;
                }
                remaining_distance = spline.get_length() - distance;

                // std::cout << GREEN << "first:         " << first_exploration_iteration << RESET << std::endl;
                // std::cout << GREEN << "Distance:      " << distance << RESET << std::endl;
                // std::cout << GREEN << "Spline length: " << spline.get_length() << RESET << std::endl;
                // std::cout << GREEN << "remaining:     " << remaining_distance << RESET << std::endl;

                if (first_exploration_iteration ||
                    (remaining_distance < TARGET_VELOCITY && error.norm < TARGET_VELOCITY)) {
                    start = interest_points[start_idx++];
                    end = interest_points[end_idx++];
                    path = find_path(start, end);
                    spline = mdi::BezierSpline(path);
                    start_time = ros::Time::now();
                    delta_time = ros::Time::now() - start_time;
                }

                std::cout << GREEN << "delta_time: " << delta_time << RESET << std::endl;

                // control the drone along the spline path
                distance = delta_time.toSec() * velocity_target;
                expected_pos = spline.get_point_at_distance(distance);

                // when object is matched, go to INSPECTION
                // if (exploration_complete) {
                //     mission_state_msg.state = INSPECTION;
                // }
                first_exploration_iteration = false;
                break;
            case INSPECTION:
                // when object paths are completed,set mission complete, go to HOME
                // follow pre-determined paths around object

                // when paths are completed, do:
                inspection_complete = true;
                mission_state_msg.state = HOME;
                break;
            case LAND:
                // request to land every 5 seconds until drone is landing
                if (drone_state.mode != "AUTO.LAND" &&
                    (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
                    mavros_msgs::CommandTOL land_msg;
                    land_msg.request.altitude = 0;

                    if (client_land.call(land_msg)) {
                        ROS_INFO("drone landing service request: success");
                    } else {
                        ROS_INFO("drone landing service request: fail");
                    }
                    previous_request_time = ros::Time::now();
                }
                // turn off this node
                break;
            default:
                ROS_WARN_STREAM("Unknown state");
                break;
        }

        // state message content
        geometry_msgs::Pose p;
        p.position.x = expected_pos.x();
        p.position.z = expected_pos.z();
        p.position.y = expected_pos.y();
        mission_state_msg.target = p;

        mission.publish();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
