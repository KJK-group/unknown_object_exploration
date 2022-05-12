#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <eigen3/Eigen/Dense>

#include "boost/format.hpp"
#include "mdi/utils/eigen.hpp"
#include "mdi/utils/rviz/rviz.hpp"
#include "mdi/utils/state.hpp"
#include "mdi/utils/transform.hpp"
#include "mdi/utils/transformlistener.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h"
#include "visualization_msgs/Marker.h"

constexpr auto VELOCITY_MAX = 5.f;
constexpr auto VELOCITY_MIN = -VELOCITY_MAX;
constexpr auto VELOCITY_MAX_YAW = 1.f;
constexpr auto VELOCITY_MIN_YAW = -VELOCITY_MAX_YAW;

// ros
ros::Publisher pub_velocity;
ros::Publisher pub_error;
ros::Publisher pub_visualisation;
ros::Subscriber sub_mission_state;
ros::Subscriber sub_odom;

// cb variables
mdi_msgs::MissionStateStamped state;
nav_msgs::Odometry odom;
tf2::Quaternion drone_attitude;

// controller gains
auto krp = 1.f;
auto kri = 0.f;
auto krd = 0.f;
auto kap = 1.f;
auto kai = 0.f;
auto kad = 0.f;

// errors
auto error_position = Eigen::Vector3f(0, 0, 0);
auto error_position_integral = Eigen::Vector3f(0, 0, 0);
auto error_position_previous = Eigen::Vector3f(0, 0, 0);
auto error_yaw = 0.f;
auto error_yaw_integral = 0.f;
auto error_yaw_previous = 0.f;

// commands
auto command_linear = Eigen::Vector3f(0, 0, 0);
// auto c_tf = Eigen::Vector3f(0, 0, 0);

// sequence counters
auto seq_command = 0;
auto seq_error = 0;

//--------------------------------------------------------------------------------------------------
// takes a 3D euclidean position error `error`,
// updates integrat and derivative errors,
// applies PID controller to produce velocity commands
auto linear_pid_velocities(Eigen::Vector3f error) -> geometry_msgs::TwistStamped {
    // update integral error if the drone is in the air
    if (odom.pose.pose.position.z > mdi::utils::SMALL_DISTANCE_TOLERANCE) {
        error_position_integral += error;
    }
    // change in error since previous time step
    auto error_derivative = error - error_position_previous;

    // linear velocity controller output
    command_linear = krp * error + kri * error_position_integral + krd * error_derivative;
    command_linear = mdi::utils::eigen::clamp_vec3(command_linear, VELOCITY_MIN, VELOCITY_MAX);

    // transform linear velocities from world frame to match drone frame

    // auto rotation_transform = mdi::utils::transform::make_transform({0, 0, 0}, {0, 0, tf2::getYaw(drone_attitude)});
    // geometry_msgs::TransformStamped rotation_transform_stamped;
    // rotation_transform_stamped.child_frame_id = mdi::utils::FRAME_BODY;
    // rotation_transform_stamped.header.frame_id = mdi::utils::FRAME_WORLD;
    // rotation_transform_stamped.transform = rotation_transform;
    // c_tf = mdi::utils::transform::transform_vec3(c, rotation_transform_stamped);
    // auto yaw = tf2::getYaw(drone_attitude);
    // Eigen::Matrix3f RzYaw;

    // RzYaw << std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1;

    // c_tf = RzYaw * c;
    // c_tf = mdi::utils::transform::convert_ned_enu(c);

    // std::cout << mdi::utils::GREEN << "COMMAND" << RESET << std::endl;
    // std::cout << mdi::utils::ITALIC << mdi::utils::BOLD << "x: " << c.x() << "\ty: " << c.y() << "\tz: " << c.z()
    //           << std::endl;

    // std::cout << mdi::utils::GREEN << "COMMAND TRANSFORMED" << RESET << std::endl;
    // std::cout << mdi::utils::ITALIC << mdi::utils::BOLD << "x: " << c_tf.x() << "\ty: " << c_tf.y()
    //           << "\tz: " << c_tf.z() << std::endl;

    // message to publish
    geometry_msgs::TwistStamped command{};
    command.header.seq = seq_command++;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = mdi::utils::FRAME_WORLD;  // FIX ME
    command.twist.linear.x = command_linear.x();
    command.twist.linear.y = command_linear.y();
    command.twist.linear.z = command_linear.z();

    // current error is the previous error for the next time step
    error_position_previous = error;
    // command_previous = command;

    return command;
}

//--------------------------------------------------------------------------------------------------
// takes a quaternion and returns the angular velocity around z,
// needed to correct the drone's yaw to turn towards the desired direction applying PID gains.
auto angular_pid_velocities(geometry_msgs::Quaternion target_attitude) -> float {
    // attitude error
    auto yaw = mdi::utils::state::clamp_yaw(tf2::getYaw(drone_attitude) + M_PI / 2);
    auto expected_yaw = tf2::getYaw(target_attitude);

    auto error_heading = mdi::utils::state::clamp_yaw(expected_yaw - yaw);

    // std::cout << mdi::utils::BOLD << mdi::utils::ITALIC << "YAW: " << yaw << "\tTARGET: " << expected_yaw
    //           << "\tERROR: " << error_heading << std::endl;

    error_yaw = error_heading;
    // update integral error if the drone is in the air
    if (odom.pose.pose.position.z > mdi::utils::SMALL_DISTANCE_TOLERANCE) {
        error_yaw_integral += error_heading;
    }
    // change in error since previous time step
    auto error_yaw_derivative = error_heading - error_yaw_previous;
    // pid output
    auto yaw_vel = kap * error_heading + kai * error_yaw_integral + kad * error_yaw_derivative;

    error_yaw_previous = error_heading;

    return std::clamp(yaw_vel, VELOCITY_MIN_YAW, VELOCITY_MAX_YAW);
}

auto sphere_msg_gen = mdi::utils::rviz::sphere_msg_gen{};
auto arrow_builder = mdi::utils::rviz::arrow_msg_gen::Builder{};
auto visualise() -> void {
    auto m = sphere_msg_gen(mdi::utils::transform::geometry_mgs_point_to_vec(odom.pose.pose.position) + error_position);
    m.header.frame_id = mdi::utils::FRAME_WORLD;

    m.color.a = 1;
    m.color.r = 1;
    m.color.g = 0;
    m.color.b = 0;

    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;

    // std::cout << mdi::utils::MAGENTA << "PUBLISHING" << RESET << std::endl;
    // std::cout << mdi::utils::ITALIC << mdi::utils::BOLD << "x: " << m.pose.position.x << "\ty: " << m.pose.position.y
    //           << "\tz: " << m.pose.position.z << std::endl;

    auto pos = mdi::utils::transform::geometry_mgs_point_to_vec(odom.pose.pose.position);
    pub_visualisation.publish(m);

    auto m2 = arrow_builder.arrow_head_width(0.05)
                  .arrow_width(0.025)
                  .arrow_length(0.05)
                  .color(0, 1, 0, 1)
                  .build()({pos, pos + command_linear});
    m2.header.frame_id = mdi::utils::FRAME_WORLD;
    pub_visualisation.publish(m2);
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
    if (argc > 1) krp = std::stof(argv[1]);
    if (argc > 2) kri = std::stof(argv[2]);
    if (argc > 3) krd = std::stof(argv[3]);
    if (argc > 4) kap = std::stof(argv[4]);
    if (argc > 5) kai = std::stof(argv[5]);
    if (argc > 6) kad = std::stof(argv[6]);

    //----------------------------------------------------------------------------------------------
    // state subscriber
    sub_mission_state =
        nh.subscribe<mdi_msgs::MissionStateStamped>("/mdi/state", mdi::utils::DEFAULT_QUEUE_SIZE, mission_state_cb);
    // odom subscriber
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", mdi::utils::DEFAULT_QUEUE_SIZE, odom_cb);

    //----------------------------------------------------------------------------------------------
    // velocity publisher
    pub_velocity =
        nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", mdi::utils::DEFAULT_QUEUE_SIZE);
    // error publisher
    pub_error = nh.advertise<mdi_msgs::PointNormStamped>("/mdi/error", mdi::utils::DEFAULT_QUEUE_SIZE);
    // visualisation publisher
    pub_visualisation =
        nh.advertise<visualization_msgs::Marker>("/mdi/visualisation/controller", mdi::utils::DEFAULT_QUEUE_SIZE);

    //----------------------------------------------------------------------------------------------
    // PID control loop
    while (ros::ok()) {
        // position error
        auto expected_position =
            Eigen::Vector3f(state.target.position.x, state.target.position.y, state.target.position.z);
        // if (auto error_opt =
        //         tf_listener.transform_vec3(mdi::utils::FRAME_BODY, mdi::utils::FRAME_WORLD, expected_position)) {
        //     // converting from NED to ENU coordinates
        //     error_position = Eigen::Vector3f(mdi::utils::transform::convert_ned_enu(error_opt.value()));
        //     visualise();
        // }
        error_position = mdi::utils::transform::geometry_mgs_point_to_vec(state.target.position) -
                         mdi::utils::transform::geometry_mgs_point_to_vec(odom.pose.pose.position);
        tf2::Quaternion drone_attitude_ned;
        tf2::fromMsg(odom.pose.pose.orientation, drone_attitude_ned);
        drone_attitude = drone_attitude_ned * tf2::Quaternion(std::sqrt(2) / 2, -std::sqrt(2) / 2, 0, 0);

        auto command_velocity = linear_pid_velocities(error_position);
        command_velocity.twist.angular.z = state.state == 2 ? angular_pid_velocities(state.target.orientation) : 0;
        // command_velocity.twist.angular.z = 0.1;
        // for debugging purposes
        // command_velocity.twist.linear.x = 0;
        // command_velocity.twist.linear.y = 0;
        // command_velocity.twist.linear.z = 0;

        // publish velocity control command
        // only if the mission state is different from LAND
        if (state.state == 0 || state.state == 1 || state.state == 2 || state.state == 3) {
            pub_velocity.publish(command_velocity);
        }
        // pub_velocity.publish(command_velocity);
        // publish error
        mdi_msgs::PointNormStamped error_msg;
        error_msg.header.seq = seq_error++;
        error_msg.header.stamp = ros::Time::now();
        error_msg.header.frame_id = mdi::utils::FRAME_BODY;
        error_msg.point.x = error_position.x();
        error_msg.point.y = error_position.y();
        error_msg.point.z = error_position.z();
        error_msg.norm = error_position.norm();
        pub_error.publish(error_msg);

        visualise();
        //------------------------------------------------------------------------------------------
        // drone position
        ROS_INFO_STREAM(mdi::utils::GREEN << mdi::utils::BOLD << mdi::utils::ITALIC
                                          << "position:" << mdi::utils::RESET);
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
        ROS_INFO_STREAM("  yaw:  " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8),
                                                           tf2::getYaw(odom.pose.pose.orientation)));
        //------------------------------------------------------------------------------------------
        // position errors
        ROS_INFO_STREAM(mdi::utils::GREEN << mdi::utils::BOLD << mdi::utils::ITALIC << "errors:" << mdi::utils::RESET);
        ROS_INFO_STREAM("  x:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error_position.x()));
        ROS_INFO_STREAM("  y:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error_position.y()));
        ROS_INFO_STREAM("  z:    " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error_position.z()));
        ROS_INFO_STREAM("  norm: " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error_position.norm()));
        ROS_INFO_STREAM("  yaw:  " << boost::format("%1.5f") %
                                          boost::io::group(std::setfill(' '), std::setw(8), error_yaw));
        //------------------------------------------------------------------------------------------
        // controller outputs
        ROS_INFO_STREAM(mdi::utils::GREEN << mdi::utils::BOLD << mdi::utils::ITALIC
                                          << "controller outputs:" << mdi::utils::RESET);
        ROS_INFO_STREAM("  x_vel:   " << boost::format("%1.5f") % boost::io::group(std::setfill(' '), std::setw(8),
                                                                                   command_velocity.twist.linear.x));
        ROS_INFO_STREAM("  y_vel:   " << boost::format("%1.5f") % boost::io::group(std::setfill(' '), std::setw(8),
                                                                                   command_velocity.twist.linear.y));
        ROS_INFO_STREAM("  z_vel:   " << boost::format("%1.5f") % boost::io::group(std::setfill(' '), std::setw(8),
                                                                                   command_velocity.twist.linear.z));
        ROS_INFO_STREAM("  norm:    " << boost::format("%1.5f") %
                                             boost::io::group(std::setfill(' '), std::setw(8),
                                                              Eigen::Vector3f(command_velocity.twist.linear.x,
                                                                              command_velocity.twist.linear.y,
                                                                              command_velocity.twist.linear.z)
                                                                  .norm()));
        ROS_INFO_STREAM("  yaw_vel: " << boost::format("%1.5f") % boost::io::group(std::setfill(' '), std::setw(8),
                                                                                   command_velocity.twist.angular.z));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}