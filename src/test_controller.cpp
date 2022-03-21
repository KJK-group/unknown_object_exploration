#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>

#include <cmath>
#include <tuple>

#define TOLERANCE 0.2

using namespace std;

ros::Publisher pub_velocity;
ros::Publisher pub_position;

ros::Subscriber sub_state;
ros::Subscriber sub_odom;

ros::ServiceClient client_arm;
ros::ServiceClient client_mode;

// state variables
mavros_msgs::State state;
auto reached_altitude = false;

auto desired_altitude = 5.f;

auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    // current position
    auto pos = msg->pose.pose.position;
    // current yaw
    auto yaw = tf2::getYaw(msg->pose.pose.orientation);

    // altitude error
    auto error_alt = abs(desired_altitude - pos.z);
    ROS_INFO("altitude error: %f.5", error_alt);

    reached_altitude = (error_alt < TOLERANCE);
}

auto state_cb(const mavros_msgs::State::ConstPtr& msg) -> void { state = *msg; }

auto main(int argc, char** argv) -> int {
    // ROS initialisations
    ros::init(argc, argv, "mdi_test_controller");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);

    // state subsbricer
    sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // odom subsbricer
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/odometry/in", 10, odom_cb);

    // velocity publisher
    pub_velocity =
        nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    // position publisher
    pub_position = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // arm service client
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // mode service client
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // wait for FCU connection
    while (ros::ok() && !state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        // arm the drone
        if (!state.armed) {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            if (client_arm.call(srv)) {
                ROS_INFO("throttle armed: success");
            } else {
                ROS_WARN("throttle armed: fail");
            }
        }

        // set drone mode to OFFBOARD
        if (state.mode != "OFFBOARD") {
            mavros_msgs::SetMode mode_msg;
            mode_msg.request.custom_mode = "OFFBOARD";

            if (client_mode.call(mode_msg) && mode_msg.response.mode_sent) {
                ROS_INFO("mode set: OFFBOARD");
            } else {
                ROS_WARN("mode set: fail");
            }
        }

        // fly to pose
        // then fly in a direction
        if (!reached_altitude) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.z = desired_altitude;
            pub_position.publish(pose);

            // ROS_INFO("flying to pose");
        } else {
            geometry_msgs::TwistStamped command;
            command.twist.linear.x = 1;
            pub_velocity.publish(command);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}