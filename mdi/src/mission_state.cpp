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
#include "mdi/rrt/rrt.hpp"
#include "mdi/rrt/rrt_builder.hpp"
#include "mdi/utils/rviz/rviz.hpp"
#include "mdi/utils/transformlistener.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h"
#include "mdi_msgs/RrtFindPath.h"

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
ros::ServiceClient client_rrt;

// time
ros::Time start_time;

// transform utilities
tf2_ros::Buffer tf_buffer;

// Bezier Spline
mdi::trajectory::BezierSpline spline;

// Points of interest
auto interest_points = std::vector<Eigen::Vector3f>{
    Eigen::Vector3f(0, 0, 5),   Eigen::Vector3f(12, 12, 7),  Eigen::Vector3f(18, 9, 15), Eigen::Vector3f(20, 25, 20),
    Eigen::Vector3f(7, 21, 13), Eigen::Vector3f(10, 10, 17), Eigen::Vector3f(0, 0, 5)};

auto find_path(Eigen::Vector3f start, Eigen::Vector3f end) -> std::vector<Eigen::Vector3f> {
    std::cout << "GOERJGEORJGERGOERGJ" << std::endl;
    const auto goal_tolerance = 2;
    auto rrt_msg = mdi_msgs::RrtFindPath{};
    rrt_msg.request.probability_of_testing_full_path_from_new_node_to_goal = 0;
    rrt_msg.request.goal_bias = 0.7;
    rrt_msg.request.goal_tolerance = goal_tolerance;
    rrt_msg.request.start.x = start.x();
    rrt_msg.request.start.y = start.y();
    rrt_msg.request.start.z = start.z();
    rrt_msg.request.goal.x = end.x();
    rrt_msg.request.goal.y = end.y();
    rrt_msg.request.goal.z = end.z();
    rrt_msg.request.max_iterations = 10000;
    rrt_msg.request.step_size = 2;

    std::vector<Eigen::Vector3f> path;
    if (client_rrt.call(rrt_msg)) {
        auto& waypoints = rrt_msg.response.waypoints;
        // path = std::vector<Eigen::Vector3f>(waypoints.size());
        std::cout << "before for loop" << std::endl;
        for (auto& wp : waypoints) {
            std::cout << wp << std::endl;
            path.emplace_back(wp.x, wp.y, wp.z);
        }
    }

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.05f)
                             .arrow_length(0.05f)
                             .arrow_width(0.05f)
                             .color({0, 1, 0, 1})
                             .build();
    arrow_msg_gen.header.frame_id = mdi::utils::FRAME_WORLD;

    auto sphere_msg_gen = mdi::utils::rviz::sphere_msg_gen{};
    sphere_msg_gen.header.frame_id = "world_enu";

    ros::Duration(1).sleep();
    // publish starting position
    auto start_msg = sphere_msg_gen(start);
    start_msg.color.r = 0.5;
    start_msg.color.g = 1;
    start_msg.color.b = 0.9;
    start_msg.color.a = 1;
    start_msg.scale.x = 0.2;
    start_msg.scale.y = 0.2;
    start_msg.scale.z = 0.2;
    pub_visualize_rrt.publish(start_msg);
    // publish goal position
    auto goal_msg = sphere_msg_gen(end);
    goal_msg.color.r = 0;
    goal_msg.color.g = 1;
    goal_msg.color.b = 0;
    goal_msg.color.a = 1;
    goal_msg.scale.x = 0.2;
    goal_msg.scale.y = 0.2;
    goal_msg.scale.z = 0.2;
    pub_visualize_rrt.publish(goal_msg);

    // visualise end tolerance
    auto sphere_tolerance_msg = sphere_msg_gen(end);

    auto msg = sphere_msg_gen(end);
    msg.scale.x = goal_tolerance * 2;
    msg.scale.y = goal_tolerance * 2;
    msg.scale.z = goal_tolerance * 2;
    msg.color.r = 1.f;
    msg.color.g = 1.f;
    msg.color.b = 1.f;
    msg.color.a = 0.2f;
    pub_visualize_rrt.publish(msg);

    // visualize result
    arrow_msg_gen.color.r = 0.0f;
    arrow_msg_gen.color.g = 1.0f;
    arrow_msg_gen.color.b = 0.0f;
    arrow_msg_gen.scale.x = 0.05f;
    arrow_msg_gen.scale.y = 0.05f;
    arrow_msg_gen.scale.z = 0.05f;
    int i = 1;
    while (ros::ok() && i < path.size()) {
        auto& p1 = path[i - 1];
        auto& p2 = path[i];
        auto arrow = arrow_msg_gen({p1, p2});
        pub_visualize_rrt.publish(arrow);
        ++i;
    }

    return path;
}

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

    mdi::trajectory::BezierSpline spline;  // = mdi::trajectory::BezierSpline(spline_input_points);

    //----------------------------------------------------------------------------------------------
    // mavros state subscriber
    sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // error subscriber
    sub_error = nh.subscribe<mdi_msgs::PointNormStamped>("/mdi/error", 10, error_cb);

    //----------------------------------------------------------------------------------------------
    // velocity publisher
    pub_mission_state = nh.advertise<mdi_msgs::MissionStateStamped>("/mdi/state", 10);

    // RRT Visualisation
    pub_visualize_rrt = nh.advertise<visualization_msgs::Marker>("/mdi/visualisation_marker", 10);

    auto sphere_msg_gen = mdi::utils::rviz::sphere_msg_gen{};
    sphere_msg_gen.header.frame_id = "world_enu";
    auto test_msg = sphere_msg_gen({0, 0, 7});
    test_msg.color.r = 0.5;
    test_msg.color.g = 1;
    test_msg.color.b = 0.9;
    test_msg.color.a = 1;
    test_msg.scale.x = 0.2;
    test_msg.scale.y = 0.2;
    test_msg.scale.z = 0.2;
    for (int i = 0; i < 5; i++) {
        std::cout << "publishing test point" << std::endl;
        pub_visualize_rrt.publish(test_msg);
        ros::spinOnce();
        rate.sleep();
    }

    //----------------------------------------------------------------------------------------------
    // arm service client
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // mode service client
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // land service client
    client_land = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    client_rrt = nh.serviceClient<mdi_msgs::RrtFindPath>("/mdi/rrt_service/find_path");

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

    // waypoint indices
    auto start_idx = 0;
    auto end_idx = 1;

    //----------------------------------------------------------------------------------------------
    // control loop
    while (ros::ok()) {
        //------------------------------------------------------------------------------------------
        // request to set drone mode to OFFBOARD every 5 seconds until successful
        if (drone_state.mode != "OFFBOARD" && (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
            if (client_mode.call(mode_msg) && mode_msg.response.mode_sent) {
                ROS_INFO("mode set: OFFBOARD");
            } else {
                ROS_INFO("mode set: fail");
            }
            previous_request_time = ros::Time::now();
        }
        //------------------------------------------------------------------------------------------
        // request to arm throttle every 5 seconds until succesful
        if (! drone_state.armed && (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
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
                    spline = mdi::trajectory::BezierSpline(path);
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
