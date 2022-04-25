#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <tuple>

#include "boost/format.hpp"
#include "mdi/bezier_spline.hpp"

#define V_MAX 0.2
#define A 0.4
#define B 0.6
#define C -1.4
#define D -0.6
#define E 1.6
#define F -1.5

using namespace std;
using boost::format;
using boost::io::group;
using Eigen::Vector2f;
using Eigen::Vector3f;
using mdi::BezierSpline;

// escape codes
auto magenta = "\u001b[35m";
auto green = "\u001b[32m";
auto reset = "\u001b[0m";
auto bold = "\u001b[1m";
auto italic = "\u001b[3m";
auto underline = "\u001b[4m";

// publishers
ros::Publisher pub_velocity;
ros::Publisher pub_point_world;
ros::Publisher pub_point_body;
ros::Publisher pub_point_est;

// subscibers
ros::Subscriber sub_state;
ros::Subscriber sub_odom;

// services
ros::ServiceClient client_arm;
ros::ServiceClient client_mode;

// time
ros::Time start_time;

// transform utilities
tf2_ros::Buffer tf_buffer;
// frames
auto frame_world = "PX4";
auto frame_body = "PX4/odom_local_ned";

// sequence counters
auto seq_point_world = 0;
auto seq_point_body = 0;
auto seq_point_est = 0;

// state variables
mavros_msgs::State state;
nav_msgs::Odometry odom;

// targets
auto altitude_offset = 5.f;
auto subject_center = Vector3f(0.0f, 0.0f, move(altitude_offset));

// controller gains
// auto k_rho = 1.f;
auto k_rho_p = 1.f;
auto k_rho_i = 1.f;
auto k_rho_d = 1.f;
auto k_alpha = 0.f;

// errors for PID
auto error_integral = Vector3f(0, 0, 0);
auto error_derivative = Vector3f(0, 0, 0);
auto error_previous = Vector3f(0, 0, 0);

// utility for PID
// auto previous_velocity = Vector3f(0, 0, 0);
// auto acceleration = Vector3f(0, 0, 0);
// auto velocity = Vector3f(0, 0, 0);

// auto acceleration_from_odom() -> Vector3f {
//     acceleration =
//         previous_velocity -
//         Vector3f(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
//         odom.twist.twist.linear.z);
//     return acceleration;
// }

// auto velocity_from_odom() -> Vector3f {
//     velocity =
//         Vector3f(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
//         odom.twist.twist.linear.z);
//     return velocity;
// }

//--------------------------------------------------------------------------------------------------
// Bezier spline
//--------------------------------------------------------------------------------------------------

BezierSpline spline;
auto spline_input_points = vector<Vector3f>{Vector3f(0.0, 0.0, 0.0),  Vector3f(3.0, 0.5, 1.0), Vector3f(-3.5, 1.5, 0.0),
                                            Vector3f(-2.8, 1.0, 0.7), Vector3f(1.2, 2.2, 1.5), Vector3f(1.0, 3.0, 1.0)};
auto forwards = true;

//--------------------------------------------------------------------------------------------------
// Polynomial Functions
//--------------------------------------------------------------------------------------------------

// 5th order trajectory function
auto trajectory(float x) -> float { return A * pow(x, 5) + B * pow(x, 4) + C * pow(x, 3) + D * pow(x, 2) + E * x + F; }

// 5th order trajectory slope
auto trajectory_slope(float x) -> float { return 5 * A * pow(x, 4) + 4 * B * pow(x, 3) + 3 * pow(x, 2) + D * x + E; }

//--------------------------------------------------------------------------------------------------
// Vector Functions
//--------------------------------------------------------------------------------------------------

auto scale = 4;

// circle vector function
auto circle_trajectory(float t) -> Vector2f { return Vector2f(scale * cos(V_MAX * t), scale * sin(V_MAX * t)); }

// 3D trajectory
auto circle_trajectory_3d(float t) -> Vector3f {
    return Vector3f(scale * cos(V_MAX * t), scale * sin(V_MAX * t), 1 * cos(t));
}

//--------------------------------------------------------------------------------------------------
// Callback Functions
//--------------------------------------------------------------------------------------------------

auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void { odom = *msg; }

auto state_cb(const mavros_msgs::State::ConstPtr& msg) -> void { state = *msg; }

auto main(int argc, char** argv) -> int {
    //----------------------------------------------------------------------------------------------
    // ROS initialisations
    ros::init(argc, argv, "mdi_test_controller");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    // save start_time
    start_time = ros::Time::now();
    //----------------------------------------------------------------------------------------------
    // transform utilities
    tf2_ros::TransformListener tf_listener(tf_buffer);

    //----------------------------------------------------------------------------------------------
    // pass in arguments
    if (argc > 1) k_rho_p = stof(argv[1]);
    if (argc > 2) k_rho_i = stof(argv[2]);
    if (argc > 3) k_rho_d = stof(argv[3]);
    if (argc > 4) k_alpha = stof(argv[4]);

    //----------------------------------------------------------------------------------------------
    // spline preprocessing
    auto spline_input_points =
        vector<Vector3f>{Vector3f(0.0, 0.0, 0.0),  Vector3f(3.0, 0.5, 1.0), Vector3f(-3.5, 1.5, 0.0),
                         Vector3f(-2.8, 1.0, 0.7), Vector3f(1.2, 2.2, 1.5), Vector3f(1.0, 3.0, 1.0)};
    for (auto& point : spline_input_points) {
        point *= 10;
    }

    spline = BezierSpline(spline_input_points);

    //----------------------------------------------------------------------------------------------
    // state subscriber
    sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // odom subscriber
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);

    //----------------------------------------------------------------------------------------------
    // velocity publisher
    pub_velocity = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    // point publishers
    pub_point_body = nh.advertise<geometry_msgs::PointStamped>("/mdi/points/expected_pos/body_frame", 10);
    pub_point_world = nh.advertise<geometry_msgs::PointStamped>("/mdi/points/expected_pos/world_frame", 10);
    pub_point_est = nh.advertise<geometry_msgs::PointStamped>("/mdi/points/estimated_pos", 10);

    //----------------------------------------------------------------------------------------------
    // arm service client
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // mode service client
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    //----------------------------------------------------------------------------------------------
    // wait for FCU connection
    while (ros::ok() && ! state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    //----------------------------------------------------------------------------------------------
    // ROS spin
    while (ros::ok()) {
        //------------------------------------------------------------------------------------------
        // control
        // current position
        auto pos = odom.pose.pose.position;
        // current yaw
        auto yaw = tf2::getYaw(odom.pose.pose.orientation);
        // time diff
        auto delta_time = (ros::Time::now() - start_time).toNSec() / pow(10, 9);

        //----------------------------------------------------------------------------------------------
        // heading error
        auto desired_heading = atan2(subject_center(1) - pos.y, subject_center(0) - pos.x);
        auto error_heading = desired_heading - yaw;
        // correct for magnitude larger than π
        if (error_heading > M_PI) {
            error_heading - 2 * M_PI;
        } else if (error_heading < -M_PI) {
            error_heading + 2 * M_PI;
        }

        //----------------------------------------------------------------------------------------------
        // lookup transform
        geometry_msgs::TransformStamped transform;
        try {
            // transform from px4 drone odom to px4 world
            transform = tf_buffer.lookupTransform(frame_body, frame_world, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_INFO("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        //----------------------------------------------------------------------------------------------
        // get expected position
        // auto expected_pos = circle_trajectory_3d(delta_time);
        // moving along the spline and back again
        auto distance_along_spline = delta_time * V_MAX;
        auto spline_length = spline.get_length();
        if (distance_along_spline >= spline_length) {
            start_time = ros::Time::now();
            forwards = ! forwards;
        }
        if (forwards) {
            distance_along_spline = spline_length - distance_along_spline;
            if (distance_along_spline < 0) {
                distance_along_spline = 0;
            }
        }

        auto expected_pos = spline.get_point_at_distance(distance_along_spline);

        //----------------------------------------------------------------------------------------------
        // transform expected_pos to point_body_frame frame
        //----------------------------------------------------------------------------------------------
        // point in world frame "PX4"
        auto point_world_frame = geometry_msgs::PointStamped();
        // fill header
        point_world_frame.header.seq = seq_point_world++;
        point_world_frame.header.stamp = ros::Time::now();
        point_world_frame.header.frame_id = frame_world;
        // fill point data
        point_world_frame.point.x = expected_pos.x();
        point_world_frame.point.y = expected_pos.y();
        point_world_frame.point.z = expected_pos.z() + altitude_offset;
        //----------------------------------------------------------------------------------------------
        // point in point_body_frame frame "PX4/odom_local_ned"
        auto point_body_frame = geometry_msgs::PointStamped();
        // fill header
        point_body_frame.header.seq = seq_point_body++;
        point_body_frame.header.stamp = ros::Time::now();
        point_body_frame.header.frame_id = frame_body;
        // apply transform outputting result to point_body_frame
        tf2::doTransform(point_world_frame, point_body_frame, transform);
        auto expected_pos_body = Vector3f(point_body_frame.point.x, point_body_frame.point.y, point_body_frame.point.z);
        //----------------------------------------------------------------------------------------------
        // publish points
        pub_point_world.publish(point_world_frame);
        pub_point_body.publish(point_body_frame);
        // pub_point_est.publish(point_est);

        //----------------------------------------------------------------------------------------------
        // position errors
        auto error = Vector3f(expected_pos_body.y(), expected_pos_body.x(), expected_pos_body.z());
        // only accumulate error if the drone is in the air
        if (pos.z > 0.1) {
            error_integral += error;
        }
        error_derivative = error - error_previous;

        // auto error_x = expected_pos(0) - pos.x;
        // auto error_y = expected_pos(1) - pos.y;
        // auto error_z = expected_pos(2) + altitude_offset - pos.z;
        // auto error_x = expected_pos_body(1);
        // auto error_y = expected_pos_body(0);
        // auto error_z = expected_pos_body(2);  //  -pos.z;

        //----------------------------------------------------------------------------------------------
        // controller
        auto omega = k_alpha * error_heading;
        auto control_vel = k_rho_p * error + k_rho_i * error_integral + k_rho_d * error_derivative;
        // auto x_vel = k_rho * error_x;
        // auto y_vel = k_rho * error_y;
        // auto z_vel = k_rho * error_z;
        // omega = 0;
        // x_vel = 0;
        // y_vel = 0;
        // z_vel = 0;
        //----------------------------------------------------------------------------------------------
        // control command
        geometry_msgs::TwistStamped command;
        command.twist.angular.z = omega;
        command.twist.linear.x = control_vel.x();
        command.twist.linear.y = control_vel.y();
        command.twist.linear.z = control_vel.z();
        pub_velocity.publish(command);

        //----------------------------------------------------------------------------------------------
        // logging for debugging
        // ROS_INFO_STREAM(magenta << "transform:\n" << transform << reset);
        ROS_INFO_STREAM(magenta << "from pose:\n" << point_world_frame << reset);
        ROS_INFO_STREAM(magenta << "to pose:\n" << point_body_frame << reset);
        // standard state logging
        ROS_INFO_STREAM(green << bold << italic << "position:" << reset);
        ROS_INFO_STREAM("  x: " << format("%1.5f") % group(setfill(' '), setw(8), pos.x));
        ROS_INFO_STREAM("  y: " << format("%1.5f") % group(setfill(' '), setw(8), pos.y));
        ROS_INFO_STREAM("  z: " << format("%1.5f") % group(setfill(' '), setw(8), pos.z));
        ROS_INFO_STREAM(green << bold << italic << "errors:" << reset);
        ROS_INFO_STREAM("  heading: " << format("%1.5f") % group(setfill(' '), setw(8), error_heading));
        ROS_INFO_STREAM("  x:       " << format("%1.5f") % group(setfill(' '), setw(8), error.x()));
        ROS_INFO_STREAM("  y:       " << format("%1.5f") % group(setfill(' '), setw(8), error.y()));
        ROS_INFO_STREAM("  z:       " << format("%1.5f") % group(setfill(' '), setw(8), error.z()));
        ROS_INFO_STREAM(green << bold << italic << "controller outputs:" << reset);
        ROS_INFO_STREAM("  omega: " << format("%1.5f") % group(setfill(' '), setw(8), omega));
        ROS_INFO_STREAM("  x_vel: " << format("%1.5f") % group(setfill(' '), setw(8), control_vel.x()));
        ROS_INFO_STREAM("  y_vel: " << format("%1.5f") % group(setfill(' '), setw(8), control_vel.y()));
        ROS_INFO_STREAM("  z_vel: " << format("%1.5f") % group(setfill(' '), setw(8), control_vel.z()));
        ROS_INFO_STREAM(green << bold << italic << "time:" << reset);
        ROS_INFO_STREAM("  delta_time: " << format("%1.2f") % group(setfill(' '), setw(5), delta_time));

        //------------------------------------------------------------------------------------------
        // arm the drone
        if (! state.armed) {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            if (client_arm.call(srv)) {
                ROS_INFO("throttle armed: success");
            } else {
                ROS_INFO("throttle armed: fail");
            }
        }
        //------------------------------------------------------------------------------------------
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
        ros::spinOnce();
        rate.sleep();
    }
    //----------------------------------------------------------------------------------------------
    return 0;
}
