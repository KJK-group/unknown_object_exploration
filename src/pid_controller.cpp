#include <ros/ros.h>

#include <cmath>
#include <eigen3/Eigen/Dense>

#include "multi_drone_inspection/MissionStateStamped.h"

#define TOLERANCE_DISTANCE 0.1

// ros
ros::Publisher pub_velocity;
ros::Subscriber sub_mission_state;
ros::Subscriber sub_odom;

// cb variables
multi_drone_inspection::MissionStateStamped state;
nav_msgs::Odometry odom;

// controller gains
auto k_rho_p = 1.f;
auto k_rho_i = 0.f;
auto k_rho_d = 0.f;
auto k_alpha_p = 1.f;
auto k_alpha_i = 0.f;
auto k_alpha_d = 0.f;

// errors
auto error_integral = Vector3f(0, 0, 0);
auto error_previous = Vector3f(0, 0, 0);

// sequence counters
auto seq_command = 0;

//--------------------------------------------------------------------------------------------------
// takes a 3D euclidean position error `error`,
// updates integrat and derivative errors,
// applies PID controller to produce velocity commands
auto pid_command_from_error(Vector3f error) -> geometry_msgs::TwistStamped {
    // update integral error if the drone is in the air
    if (odom.pose.pose.position.z > TOLERANCE_DISTANCE) {
        error_integral += error;
    }
    // change in error since previous time step
    auto error_derivative = error - error_previous;

    // linear velocity controller output
    auto c = k_rho_p * error + k_rho_p * error_integral + k_rho_p * error_derivative;
    // TODO: angular velocity controller output

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
auto mission_state_cb(const multi_drone_inspection::MissionStateStamped::ConstPtr& msg) -> void {
    state = *msg;
}

//--------------------------------------------------------------------------------------------------
// Main Function
auto main(int argc, char** argv) -> int {
    //----------------------------------------------------------------------------------------------
    // ROS initialisations
    ros::init(argc, argv, "mdi_mission_state_msg");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);

    //----------------------------------------------------------------------------------------------
    // pass in arguments
    // TODO: seperate pid gains for the different velocities
    if (argc > 1) k_rho_p = stof(argv[1]);
    if (argc > 2) k_rho_i = stof(argv[2]);
    if (argc > 3) k_rho_d = stof(argv[3]);
    if (argc > 4) k_alpha_p = stof(argv[4]);
    if (argc > 4) k_alpha_i = stof(argv[4]);
    if (argc > 4) k_alpha_d = stof(argv[4]);

    //----------------------------------------------------------------------------------------------
    // state subscriber
    sub_mission_state = nh.subscribe<multi_drone_inspection::MissionStateStamped>("/mdi/state", 10,
                                                                                  mission_state_cb);
    // odom subscriber
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);

    //----------------------------------------------------------------------------------------------
    // velocity publisher
    pub_velocity =
        nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    //----------------------------------------------------------------------------------------------
    // PID control loop
    while (ros::ok()) {
        auto error =
            eigen::Vector3f(state.pose.position.x, state.pose.position.y, state.pose.position.z);
        auto command_velocity = pid_command_from_error(error);
        pub_velocity.publish(command_velocity);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}