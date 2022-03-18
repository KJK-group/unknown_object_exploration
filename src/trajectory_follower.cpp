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
#define A 0.4
#define B 0.6
#define C -1.4
#define D -0.6
#define E 1.6
#define F -1.5

using namespace std;

ros::Publisher pub_velocity;

ros::Subscriber sub_state;
ros::Subscriber sub_odom;

ros::ServiceClient client_arm;
ros::ServiceClient client_mode;

// state variables
mavros_msgs::State state;

// targets
auto desired_altitude = 5.f;
auto subject_center = make_tuple<float, float, float>(0.0f, 0.0f, move(desired_altitude));

// controller gains
auto k_alpha = 1.f;
auto k_rho = 1.f;

// trajectory function
auto trajectory(float x) -> float {
    return A * pow(x, 5) + B * pow(x, 4) + C * pow(x, 3) + D * pow(x, 2) + E * x + F;
}

// trajectory slope
auto trajectory_slope(float x) -> float {
    return 5 * A * pow(x, 4) + 4 * B * pow(x, 3) + 3 * pow(x, 2) + D * x + E;
}

auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    // current position
    auto pos = msg->pose.pose.position;
    // current yaw
    auto yaw = tf2::getYaw(msg->pose.pose.orientation);

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
    // y
    auto error_y = trajectory(pos.x) - pos.y;
    // z - altitude error
    auto error_alt = desired_altitude - msg->pose.pose.position.z;

    // controller
    auto omega =

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

    // ROS spin
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}