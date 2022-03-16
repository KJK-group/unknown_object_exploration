#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

#define TOLERANCE 0.2

using namespace std;

// target altitude
auto desired_altitude = 5.f;

ros::Publisher pub_velocity;
ros::Publisher pub_position;
//ros::Publisher pub_mode;

ros::Subscriber sub_state;
ros::Subscriber sub_odom;

ros::ServiceClient client_arm;
ros::ServiceClient client_mode;
ros::ServiceClient client_land;

// state variables
mavros_msgs::State state;
auto reached_altitude = false;
auto armed = false;

auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    reached_altitude = (abs(desired_altitude - msg->pose.pose.position.z) < TOLERANCE);
}

auto state_cb(const mavros_msgs::State::ConstPtr& msg) -> void {
    state = *msg;
}

auto main(int argc, char** argv) -> int {
    // ROS initialisations
    ros::init(argc, argv, "mdi_test_controller");
    auto nh = ros::NodeHandle("~");
    ros::Rate rate(20.0);

    // state subsbricer
    sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // odom subsbricer
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/odometry/in", 10, odom_cb);

    // velocity publisher
    pub_velocity = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    // position publisher
    pub_position = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // arm service client
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    if (!armed) {
        // arm drone
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (client_arm.call(srv)) {
            ROS_INFO("throttle armed: success");
        }
        else {
            ROS_WARN("throttle armed: fail");
        }
    }
    
    // set drone mode to OFFBOARD
    if (state.mode != "OFFBOARD") {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if (client_arm.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("mode set: OFFBOARD");
        }
        else {
            ROS_WARN("mode set: fail");
        }
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = desired_altitude;
    pub_position.publish(pose);

    ROS_INFO("flying to pose");

    // check tolerance
    // give linear velocity

    while (ros::ok() && !reached_altitude) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped command;
    command.twist.linear.x = 1;
    pub_velocity.publish(command);

    return 0;
}

// rosrun mavros mavsys node -c OFFBOARD
// rosservice call /mavros/cmd/arming "value: true" - /mavros/cmd/arming
// rosservice call /mavros/cmd/land "{}" - /mavros/cmd/land

// set posistion
// rostopic pub -r 10 /mavros/setpoint_position/local geometry_msgs/PoseStamped "{pose: {position: {x: 10, y: 7, z: 5}}}"

// set velocity
// /mavros/setpoint_velocity/local