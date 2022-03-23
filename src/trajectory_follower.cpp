#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <tuple>

#define V_MAX 5
#define TOLERANCE 0.2
#define A 0.4
#define B 0.6
#define C -1.4
#define D -0.6
#define E 1.6
#define F -1.5

using namespace std;
using Eigen::Vector2f;

// publishers
ros::Publisher pub_velocity;

// subscibers
ros::Subscriber sub_state;
ros::Subscriber sub_odom;

// services
ros::ServiceClient client_arm;
ros::ServiceClient client_mode;

ros::Time start_time;

// transform utilities
tf2_ros::Buffer tf_buffer;

// state variables
mavros_msgs::State state;
Vector2f direction_vec;

// targets
auto desired_altitude = 5.f;
auto subject_center = make_tuple(0.0f, 0.0f, move(desired_altitude));

// controller gains
auto k_alpha = 10.f;
auto k_rho = 1.f;

// trajectory function
auto trajectory(float x) -> float {
    return A * pow(x, 5) + B * pow(x, 4) + C * pow(x, 3) + D * pow(x, 2) + E * x + F;
}

// trajectory slope
auto trajectory_slope(float x) -> float {
    return 5 * A * pow(x, 4) + 4 * B * pow(x, 3) + 3 * pow(x, 2) + D * x + E;
}

auto circle_trajectory(float t) -> Vector2f { return Vector2f(4 * cos(t), 4 * sin(t)); }

auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    // current position
    auto pos = msg->pose.pose.position;
    // current yaw
    auto yaw = tf2::getYaw(msg->pose.pose.orientation);
    // time diff
    auto delta_time = (ros::Time::now() - start_time).toNSec() / pow(10, 9);

    // heading error
    auto desired_heading = M_PI / 2;
    auto error_heading = desired_heading - yaw;
    // correct for magnitude larger than π
    if (error_heading > M_PI) {
        error_heading - 2 * M_PI;
    } else if (error_heading < -M_PI) {
        error_heading + 2 * M_PI;
    }

    // position errors
    auto expected_pos = circle_trajectory(delta_time);
    auto error_x = expected_pos(0) - pos.x;
    auto error_y = expected_pos(1) - pos.y;
    auto error_z = desired_altitude - msg->pose.pose.position.z;

    // lookup transform
    // try {
    // }

    // controller
    auto omega = k_alpha * error_heading;
    auto x_vel = k_rho * error_x;
    auto y_vel = k_rho * error_y;
    auto z_vel = k_rho * error_z;

    // control command
    geometry_msgs::TwistStamped command;
    // command.twist.angular.z = omega;
    command.twist.linear.x = x_vel;
    command.twist.linear.y = y_vel;
    command.twist.linear.z = z_vel;
    pub_velocity.publish(command);

    // logging for debugging
    ROS_WARN("errors:");
    ROS_INFO("  x: %.5f", error_x);
    ROS_INFO("  y: %.5f", error_y);
    ROS_INFO("  z: %.5f", error_z);
    ROS_WARN("controller outputs:");
    ROS_INFO("  omega: %.5f", omega);
    ROS_INFO("  x_vel: %.5f", x_vel);
    ROS_INFO("  y_vel: %.5f", y_vel);
    ROS_INFO("  z_vel: %.5f", z_vel);
    ROS_WARN("time:");
    ROS_INFO("  delta_time: %.5f", delta_time);
}

auto state_cb(const mavros_msgs::State::ConstPtr& msg) -> void { state = *msg; }

auto main(int argc, char** argv) -> int {
    // ROS initialisations
    ros::init(argc, argv, "mdi_test_controller");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    // save start_time
    start_time = ros::Time::now();

    // transform utilities
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // state subsbricer
    sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // odom subsbricer
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);

    // velocity publisher
    pub_velocity =
        nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    // arm service client
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // mode service client
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // wait for FCU connection
    while (ros::ok() && !state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // arm the drone
    if (!state.armed) {
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (client_arm.call(srv)) {
            ROS_INFO("throttle armed: success");
        } else {
            ROS_INFO("throttle armed: fail");
        }
    }

    // set drone mode to OFFBOARD
    if (state.mode != "OFFBOARD") {
        mavros_msgs::SetMode mode_msg;
        mode_msg.request.custom_mode = "OFFBOARD";

        if (client_mode.call(mode_msg) && mode_msg.response.mode_sent) {
            ROS_INFO("mode set: OFFBOARD");
        } else {
            ROS_INFO("mode set: fail");
        }
    }

    // ROS spin
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}