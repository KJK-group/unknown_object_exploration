#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "boost/format.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h"
#include "multi_drone_inspection/bezier_spline.hpp"
#include "multi_drone_inspection/utils/transformlistener.hpp"

#define TOLERANCE_DISTANCE 0.1
#define TARGET_VELOCITY 1.f              // move drone at 5m/s
#define FRAME_WORLD "world_enu"          // world/global frame
#define FRAME_BODY "PX4/odom_local_ned"  // drone body frame
#define SPLINE_Z_OFFSET 5

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

// home pose
auto home = Eigen::Vector3f(0, 0, SPLINE_Z_OFFSET);
auto expected_pos = Eigen::Vector3f(0, 0, SPLINE_Z_OFFSET);

//--------------------------------------------------------------------------------------------------
// ROS
//--------------------------------------------------------------------------------------------------

// publishers
ros::Publisher pub_mission_state;

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

//--------------------------------------------------------------------------------------------------
// Bezier Spline
//--------------------------------------------------------------------------------------------------

mdi::BezierSpline spline;
auto spline_input_points =
    vector<Eigen::Vector3f>{Eigen::Vector3f(0.0, 0.0, 0.0),  Eigen::Vector3f(3.0, 0.5, 1.0),
                            Eigen::Vector3f(-3.5, 1.5, 0.0), Eigen::Vector3f(-2.8, 1.0, 0.7),
                            Eigen::Vector3f(1.2, 2.2, 1.5),  Eigen::Vector3f(1.0, 3.0, 1.0)};
auto forwards = true;

//--------------------------------------------------------------------------------------------------
// Callback Functions
//--------------------------------------------------------------------------------------------------

auto state_cb(const mavros_msgs::State::ConstPtr& msg) -> void { drone_state = *msg; }
auto error_cb(const mdi_msgs::PointNormStamped::ConstPtr& msg) -> void { error = *msg; }
// auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void { odom = *msg; }

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
    start_time = ros::Time::now();

    //----------------------------------------------------------------------------------------------
    // transform utilities
    auto tf_listener = mdi::utils::transform::TransformListener{};

    //----------------------------------------------------------------------------------------------
    // pass in arguments
    if (argc > 1) velocity_target = std::stof(argv[1]);

    //----------------------------------------------------------------------------------------------
    // spline preprocessing
    auto spline_input_points = vector<Vector3f>{Vector3f(0.0, 0.0, 0.0),   Vector3f(3.0, 0.5, 1.0),
                                                Vector3f(-3.5, 1.5, 0.0),  Vector3f(-2.8, 1.0, 0.7),
                                                Vector3f(1.2, 2.2, 1.5),   Vector3f(1.0, 3.0, 1.0),
                                                Vector3f(-0.2, -0.5, -0.2)};
    for (auto& point : spline_input_points) {
        point *= 10;
    }

    spline = mdi::BezierSpline(spline_input_points);

    //----------------------------------------------------------------------------------------------
    // state subscriber
    sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // error subscriber
    sub_error = nh.subscribe<mdi_msgs::PointNormStamped>("/mdi/error", 10, error_cb);

    //----------------------------------------------------------------------------------------------
    // velocity publisher
    pub_mission_state = nh.advertise<mdi_msgs::MissionStateStamped>("/mdi/state", 10);

    //----------------------------------------------------------------------------------------------
    // arm service client
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // mode service client
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // land service client
    client_land = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    //----------------------------------------------------------------------------------------------
    // wait for FCU connection
    while (ros::ok() && ! drone_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    auto previous_request_time = ros::Time(0);
    // offboard mode message
    mavros_msgs::SetMode mode_msg{};
    mode_msg.request.custom_mode = "OFFBOARD";
    // arm message
    mavros_msgs::CommandBool srv{};
    srv.request.value = true;

    //----------------------------------------------------------------------------------------------
    // control loop
    while (ros::ok()) {
        //------------------------------------------------------------------------------------------
        // request to set drone mode to OFFBOARD every 5 seconds until successful
        if (drone_state.mode != "OFFBOARD" &&
            (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
            if (client_mode.call(mode_msg) && mode_msg.response.mode_sent) {
                ROS_INFO("mode set: OFFBOARD");
            } else {
                ROS_INFO("mode set: fail");
            }
            previous_request_time = ros::Time::now();
        }
        //------------------------------------------------------------------------------------------
        // request to arm throttle every 5 seconds until succesful
        if (! drone_state.armed &&
            (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
            if (client_arm.call(srv)) {
                ROS_INFO("throttle armed: success");
            } else {
                ROS_INFO("throttle armed: fail");
            }
            previous_request_time = ros::Time::now();
        }

        //------------------------------------------------------------------------------------------
        // state message header
        mission_state_msg.header.seq = seq_state++;
        mission_state_msg.header.stamp = ros::Time::now();
        mission_state_msg.header.frame_id = FRAME_WORLD;

        // transform to get error
        // TODO: publish error from controller, subscribe to it here, and use that
        // if (auto opt = tf_listener.transform_vec3(FRAME_BODY, FRAME_WORLD, expected_pos)) {
        //     error = opt.value();
        // }
        auto delta_time = ros::Time::now() - start_time;

        switch (mission_state_msg.state) {
            case PASSIVE:
                mission_state_msg.state = HOME;
                break;
            case HOME:
                // 1. if within tolerance, go to EXPLORATION
                // 2. if it's the second time we're here, go to LAND
                expected_pos = home;
                ros::Duration(1.0).sleep();
                std::cout << MAGENTA << error << RESET << std::endl;
                if (error.norm < TOLERANCE_DISTANCE && delta_time.toSec() > 5) {
                    if (inspection_complete) {
                        mission_state_msg.state = LAND;
                    } else {
                        start_time = ros::Time::now();
                        mission_state_msg.state = EXPLORATION;
                    }
                }
                break;
            case EXPLORATION:
                // when object is matched, go to INSPECTION
                // go through spline here, getting spline from BezierSpline getting input from RRT*
                expected_pos = spline.get_point_at_distance(delta_time.toSec() * velocity_target);
                expected_pos(2) = expected_pos.z() + SPLINE_Z_OFFSET;
                if (exploration_complete) {
                    mission_state_msg.state = INSPECTION;
                }
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
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "mission:" << RESET);
        ROS_INFO_STREAM("  time:   " << boost::format("%1.2f") %
                                            boost::io::group(std::setfill(' '), std::setw(w + 2),
                                                             delta_time.toSec()));
        ROS_INFO_STREAM("  dstate: " << boost::format("%s") % boost::io::group(std::setfill(' '),
                                                                               std::setw(w),
                                                                               drone_state.mode));
        ROS_INFO_STREAM(
            "  mstate: " << boost::format("%d") %
                                boost::io::group(std::setfill(' '), std::setw(w),
                                                 state_to_string((state)mission_state_msg.state)));
        ROS_INFO_STREAM("  target:");
        ROS_INFO_STREAM("    x: " << boost::format("%1.5f") %
                                         boost::io::group(std::setfill(' '), std::setw(8),
                                                          mission_state_msg.target.position.x));
        ROS_INFO_STREAM("    y: " << boost::format("%1.5f") %
                                         boost::io::group(std::setfill(' '), std::setw(8),
                                                          mission_state_msg.target.position.y));
        ROS_INFO_STREAM("    z: " << boost::format("%1.5f") %
                                         boost::io::group(std::setfill(' '), std::setw(8),
                                                          mission_state_msg.target.position.z));
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
        // ROS_INFO_STREAM("  x_vel: " << boost::format("%1.5f") % boost::io::group(std::setfill('
        // '), std::setw(8),
        //                                                        command_previous.twist.linear.x));
        // ROS_INFO_STREAM("  y_vel: " << boost::format("%1.5f") % boost::io::group(std::setfill('
        // '), std::setw(8),
        //                                                        command_previous.twist.linear.y));
        // ROS_INFO_STREAM("  z_vel: " << boost::format("%1.5f") % boost::io::group(std::setfill('
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
        // ROS_INFO_STREAM("  x:    " << boost::format("%1.5f") % boost::io::group(std::setfill('
        // '), std::setw(8), velocity.x())); ROS_INFO_STREAM("  y:    " << boost::format("%1.5f") %
        // boost::io::group(std::setfill(' '), std::setw(8), velocity.y())); ROS_INFO_STREAM("  z: "
        // << boost::format("%1.5f") % boost::io::group(std::setfill('
        // '), std::setw(8), velocity.z())); ROS_INFO_STREAM("  norm: " << boost::format("%1.5f") %
        //                                   boost::io::group(std::setfill(' '), std::setw(8),
        //                                   velocity.norm()));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
