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

// escape codes
constexpr auto MAGENTA = "\u001b[35m";
constexpr auto GREEN = "\u001b[32m";
constexpr auto RESET = "\u001b[0m";
constexpr auto BOLD = "\u001b[1m";
constexpr auto ITALIC = "\u001b[3m";
constexpr auto UNDERLINE = "\u001b[4m";

//--------------------------------------------------------------------------------------------------
// State Utilities
//--------------------------------------------------------------------------------------------------
// mission states
enum state { PASSIVE, HOME, EXPLORATION, INSPECTION, LAND };
auto state_to_string(state s) -> std::string {
    switch (s) {
        case PASSIVE:
            return "PASSIVE";
            break;
        case HOME:
            return "HOME";
            break;
        case EXPLORATION:
            return "EXPLORATION";
            break;
        case INSPECTION:
            return "INSPECTION";
            break;
        case LAND:
            return "LAND";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

// state variables
mavros_msgs::State drone_state;
nav_msgs::Odometry odom;
mdi_msgs::MissionStateStamped mission_state_msg;
mdi_msgs::PointNormStamped error;
auto velocity_target = TARGET_VELOCITY;
// mission_state_msg.state = PASSIVE;
auto seq_state = 0;
auto exploration_complete = false;
auto inspection_complete = false;
auto first_exploration_iteration = true;

// home pose
auto home = Eigen::Vector3f(0, 0, SPLINE_Z_OFFSET);
auto expected_pos = Eigen::Vector3f(0, 0, SPLINE_Z_OFFSET);

// mission instance
mdi::Mission mission;

//--------------------------------------------------------------------------------------------------
// ROS
//--------------------------------------------------------------------------------------------------

// publishers
ros::Publisher pub_mission_state;
ros::Publisher pub_visualize_rrt;

// subscribers
ros::Subscriber sub_state;
ros::Subscriber sub_error;

// services
ros::ServiceClient client_arm;
ros::ServiceClient client_mode;
ros::ServiceClient client_land;

// time
ros::Time start_time;

// transform utilities
tf2_ros::Buffer tf_buffer;

// Bezier Spline
mdi::BezierSpline spline;

// Points of interest
auto interest_points = std::vector<Eigen::Vector3f>{
    Eigen::Vector3f(0, 0, 5),   Eigen::Vector3f(10, 10, 7),  Eigen::Vector3f(18, 9, 15), Eigen::Vector3f(20, 25, 20),
    Eigen::Vector3f(7, 21, 13), Eigen::Vector3f(10, 10, 17), Eigen::Vector3f(0, 0, 5)};

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
        if (ros::Time::now() - previous_request_time > ros::Duration(5.0)) {
            mission.drone_takeoff();
            previous_request_time = ros::Time::now();
        }
        //------------------------------------------------------------------------------------------
        // request to arm throttle every 5 seconds until succesful
        if (ros::Time::now() - previous_request_time > ros::Duration(5.0)) {
            mission.drone_arm();
            previous_request_time = ros::Time::now();
        }

        auto delta_time = ros::Time::now() - start_time;
        float distance;
        float remaining_distance;
        Eigen::Vector3f start;
        Eigen::Vector3f end;
        std::vector<Eigen::Vector3f> path;

        switch (mission_state_msg.state) {
            case PASSIVE:
                mission_state_msg.state = HOME;
                break;
            case HOME:
                // 1. if within tolerance, go to EXPLORATION
                // 2. if it's the second time we're here, go to LAND
                expected_pos = home;
                ros::Duration(1.0).sleep();
                // std::cout << MAGENTA << error << RESET << std::endl;
                if (error.norm < TOLERANCE_DISTANCE && delta_time.toSec() > 5) {
                    if (inspection_complete) {
                        mission_state_msg.state = LAND;
                    } else {
                        // start_time = ros::Time::now();
                        first_exploration_iteration = true;
                        mission_state_msg.state = EXPLORATION;
                    }
                }
                break;
            case EXPLORATION:
                if (end_idx > interest_points.size()) {
                    exploration_complete = true;
                    mission_state_msg.state = INSPECTION;
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

        pub_mission_state.publish(mission_state_msg);

        int w;
        int time = delta_time.toSec();
        for (w = 0; time > 0; w++) {
            time /= 10;
        }
        //------------------------------------------------------------------------------------------
        // ROS logging
        //------------------------------------------------------------------------------------------
        // mission state
        // ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "mission:" << RESET);
        // ROS_INFO_STREAM("  time:   " << boost::format("%1.2f") %
        //                                     boost::io::group(std::setfill(' '), std::setw(w +
        //                                     2),
        //                                                      delta_time.toSec()));
        // ROS_INFO_STREAM("  dstate: " << boost::format("%s") % boost::io::group(std::setfill('
        // '),
        //                                                                        std::setw(w),
        //                                                                        drone_state.mode));
        // ROS_INFO_STREAM(
        //     "  mstate: " << boost::format("%d") %
        //                         boost::io::group(std::setfill(' '), std::setw(w),
        //                                          state_to_string((state)mission_state_msg.state)));
        // ROS_INFO_STREAM("  target:");
        // ROS_INFO_STREAM("    x: " << boost::format("%1.5f") %
        //                                  boost::io::group(std::setfill(' '), std::setw(8),
        //                                                   mission_state_msg.target.position.x));
        // ROS_INFO_STREAM("    y: " << boost::format("%1.5f") %
        //                                  boost::io::group(std::setfill(' '), std::setw(8),
        //                                                   mission_state_msg.target.position.y));
        // ROS_INFO_STREAM("    z: " << boost::format("%1.5f") %
        //                                  boost::io::group(std::setfill(' '), std::setw(8),
        //                                                   mission_state_msg.target.position.z));
        // //------------------------------------------------------------------------------------------
        // // drone position
        // ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "position:" << RESET);
        // ROS_INFO_STREAM("  x:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                                    odom.pose.pose.position.x));
        // ROS_INFO_STREAM("  y:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                                    odom.pose.pose.position.y));
        // ROS_INFO_STREAM("  z:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                                    odom.pose.pose.position.z));
        // ROS_INFO_STREAM("  norm: " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                                    Vector3f(odom.pose.pose.position.x,
        //                                                             odom.pose.pose.position.y,
        //                                                             odom.pose.pose.position.z)
        //                                                        .norm()));
        // //------------------------------------------------------------------------------------------
        // // position errors
        // ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "errors:" << RESET);
        // ROS_INFO_STREAM("  x:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   error_previous.x()));
        // ROS_INFO_STREAM("  y:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   error_previous.y()));
        // ROS_INFO_STREAM("  z:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   error_previous.z()));
        // ROS_INFO_STREAM("  norm: " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   error_previous.norm()));
        // //------------------------------------------------------------------------------------------
        // // controller outputs
        // ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "controller outputs:" << RESET);
        // ROS_INFO_STREAM("  x_vel: " << boost::format("%1.5f") %
        // boost::io::group(std::setfill('
        // '), std::setw(8),
        //                                                        command_previous.twist.linear.x));
        // ROS_INFO_STREAM("  y_vel: " << boost::format("%1.5f") %
        // boost::io::group(std::setfill('
        // '), std::setw(8),
        //                                                        command_previous.twist.linear.y));
        // ROS_INFO_STREAM("  z_vel: " << boost::format("%1.5f") %
        // boost::io::group(std::setfill('
        // '), std::setw(8),
        //                                                        command_previous.twist.linear.z));
        // ROS_INFO_STREAM("  norm:  " << boost::format("%1.5f") %
        //                                    boost::io::group(std::setfill(' '), std::setw(8),
        //                                          Vector3f(command_previous.twist.linear.x,
        //                                                   command_previous.twist.linear.y,
        //                                                   command_previous.twist.linear.z)
        //                                              .norm()));
        // //------------------------------------------------------------------------------------------
        // // acceleration
        // ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "acceleration:" << RESET);
        // ROS_INFO_STREAM("  x:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   acceleration.x()));
        // ROS_INFO_STREAM("  y:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   acceleration.y()));
        // ROS_INFO_STREAM("  z:    " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   acceleration.z()));
        // ROS_INFO_STREAM("  norm: " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   acceleration.norm()));
        // //------------------------------------------------------------------------------------------
        // // velocity
        // ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "velocity:" << RESET);
        // ROS_INFO_STREAM("  x:    " << boost::format("%1.5f") %
        // boost::io::group(std::setfill('
        // '), std::setw(8), velocity.x())); ROS_INFO_STREAM("  y:    " <<
        // boost::format("%1.5f") % boost::io::group(std::setfill(' '), std::setw(8),
        // velocity.y())); ROS_INFO_STREAM("  z: "
        // << boost::format("%1.5f") % boost::io::group(std::setfill('
        // '), std::setw(8), velocity.z())); ROS_INFO_STREAM("  norm: " <<
        // boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   velocity.norm()));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
