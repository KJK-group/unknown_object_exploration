#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cmath>
#include <eigen3/Eigen/Dense>

#include "boost/format.hpp"
#include "mdi/utils/transformlistener.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h"

#define TOLERANCE_DISTANCE 0.1
#define FRAME_WORLD "world_enu"          // world/global frame
#define FRAME_BODY "PX4/odom_local_ned"  // drone body frame

// escape codes
constexpr auto MAGENTA = "\u001b[35m";
constexpr auto GREEN = "\u001b[32m";
constexpr auto RESET = "\u001b[0m";
constexpr auto BOLD = "\u001b[1m";
constexpr auto ITALIC = "\u001b[3m";
constexpr auto UNDERLINE = "\u001b[4m";

// ros
ros::Publisher pub_velocity;
ros::Publisher pub_error;
ros::Subscriber sub_mission_state;
ros::Subscriber sub_odom;

// cb variables
mdi_msgs::MissionStateStamped state;
nav_msgs::Odometry odom;

// controller gains
auto k_rho_p = 1.f;
auto k_rho_i = 0.f;
auto k_rho_d = 0.f;
auto k_alpha_p = 1.f;
auto k_alpha_i = 0.f;
auto k_alpha_d = 0.f;

// errors
auto error = Eigen::Vector3f(0, 0, 0);
auto error_integral = Eigen::Vector3f(0, 0, 0);
auto error_previous = Eigen::Vector3f(0, 0, 0);

// sequence counters
auto seq_command = 0;
auto seq_error = 0;

//--------------------------------------------------------------------------------------------------
// takes a 3D euclidean position error `error`,
// updates integrat and derivative errors,
// applies PID controller to produce velocity commands
auto pid_command_from_error(Eigen::Vector3f error) -> geometry_msgs::TwistStamped {
    // update integral error if the drone is in the air
    if (odom.pose.pose.position.z > TOLERANCE_DISTANCE) {
        error_integral += error;
    }
    // change in error since previous time step
    auto error_derivative = error - error_previous;

    // linear velocity controller output
    auto c = k_rho_p * error + k_rho_i * error_integral + k_rho_d * error_derivative;
    // TODO: angular velocity controller output
    // std::cout << c << std::endl;

    // message to publish
    geometry_msgs::TwistStamped command{};
    command.header.seq = seq_command++;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "odom";  // FIX ME
    command.twist.linear.x = c.x();
    command.twist.linear.y = c.y();
    command.twist.linear.z = c.z();

    // current error is the previous error for the next time step
    error_previous = error;
    // command_previous = command;

    return command;
}

//--------------------------------------------------------------------------------------------------
// Callback Functions
auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void { odom = *msg; }
auto mission_state_cb(const mdi_msgs::MissionStateStamped::ConstPtr& msg) -> void { state = *msg; }

//--------------------------------------------------------------------------------------------------
// Main Function
auto main(int argc, char** argv) -> int {
    //----------------------------------------------------------------------------------------------
    // ROS initialisations
    ros::init(argc, argv, "mdi_pid_controller");
    auto nh = ros::NodeHandle();
    ros::Rate rate(mdi::utils::DEFAULT_LOOP_RATE * 2);

    //----------------------------------------------------------------------------------------------
    // transform utilities
    auto tf_listener = mdi::utils::transform::TransformListener{};

    //----------------------------------------------------------------------------------------------
    // pass in arguments
    // TODO: seperate pid gains for the different velocities
    if (argc > 1) k_rho_p = std::stof(argv[1]);
    if (argc > 2) k_rho_i = std::stof(argv[2]);
    if (argc > 3) k_rho_d = std::stof(argv[3]);
    if (argc > 4) k_alpha_p = std::stof(argv[4]);
    if (argc > 4) k_alpha_i = std::stof(argv[4]);
    if (argc > 4) k_alpha_d = std::stof(argv[4]);

    //----------------------------------------------------------------------------------------------
    // state subscriber
    sub_mission_state = nh.subscribe<mdi_msgs::MissionStateStamped>("/mdi/state", 10, mission_state_cb);
    // odom subscriber
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);

    //----------------------------------------------------------------------------------------------
    // velocity publisher
    pub_velocity = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    // error publisher
    pub_error = nh.advertise<mdi_msgs::PointNormStamped>("/mdi/error", 10);

    //----------------------------------------------------------------------------------------------
    // PID control loop
    while (ros::ok()) {
        // TODO: only do something if the mission state is different from PASSIVE
        auto expected_position =
            Eigen::Vector3f(state.target.position.x, state.target.position.y, state.target.position.z);
        if (auto error_opt = tf_listener.transform_vec3(FRAME_BODY, FRAME_WORLD, expected_position)) {
            // converting from NED to ENU coordinates
            error = Eigen::Vector3f(error_opt.value().y(), error_opt.value().x(), -error_opt.value().z());
        }
        auto command_velocity = pid_command_from_error(error);

        // publish velocity control command
        if (state.state == 0 || state.state == 1 || state.state == 2 || state.state == 3) {
            pub_velocity.publish(command_velocity);
        }
        // pub_velocity.publish(command_velocity);
        // publish error
        mdi_msgs::PointNormStamped error_msg;
        error_msg.header.seq = seq_error++;
        error_msg.header.stamp = ros::Time::now();
        error_msg.header.frame_id = FRAME_BODY;
        error_msg.point.x = error.x();
        error_msg.point.y = error.y();
        error_msg.point.z = error.z();
        error_msg.norm = error.norm();
        pub_error.publish(error_msg);

        //------------------------------------------------------------------------------------------
        // drone position
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "position:" << RESET);
        ROS_INFO_STREAM("  x:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), odom.pose.pose.position.x));
        ROS_INFO_STREAM("  y:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), odom.pose.pose.position.y));
        ROS_INFO_STREAM("  z:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), odom.pose.pose.position.z));
        ROS_INFO_STREAM(
            "  norm: " << boost::format("%1.5f") %
                              boost::io::group(std::setfill(' '), std::setw(8),
                                               Eigen::Vector3f(odom.pose.pose.position.x, odom.pose.pose.position.y,
                                                               odom.pose.pose.position.z)
                                                   .norm()));
        //------------------------------------------------------------------------------------------
        // position errors
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "errors:" << RESET);
        ROS_INFO_STREAM("  x:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error.x()));
        ROS_INFO_STREAM("  y:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error.y()));
        ROS_INFO_STREAM("  z:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error.z()));
        ROS_INFO_STREAM("  norm: " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error.norm()));
        //------------------------------------------------------------------------------------------
        // controller outputs
        ROS_INFO_STREAM(GREEN << BOLD << ITALIC << "controller outputs:" << RESET);
        ROS_INFO_STREAM("  x_vel: " << boost::format("%1.5f") % boost::io::group(std::setfill(' '), std::setw(8),
                                                                                 command_velocity.twist.linear.x));
        ROS_INFO_STREAM("  y_vel: " << boost::format("%1.5f") % boost::io::group(std::setfill(' '), std::setw(8),
                                                                                 command_velocity.twist.linear.y));
        ROS_INFO_STREAM("  z_vel: " << boost::format("%1.5f") % boost::io::group(std::setfill(' '), std::setw(8),
                                                                                 command_velocity.twist.linear.z));
        ROS_INFO_STREAM("  norm:  " << boost::format("%1.5f") %
                                           boost::io::group(std::setfill(' '), std::setw(8),
                                                            Eigen::Vector3f(command_velocity.twist.linear.x,
                                                                            command_velocity.twist.linear.y,
                                                                            command_velocity.twist.linear.z)
                                                                .norm()));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}