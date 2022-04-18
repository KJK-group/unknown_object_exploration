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
#include "multi_drone_inspection/MissionStateStamped.h"
#include "multi_drone_inspection/bezier_spline.hpp"
#include "utils/transformlistener.hpp"

#define TOLERANCE_DIST 0.1
#define TARGET_VELOCITY 5f                // move drone at 5m/s
#define FRAME_WORLD "PX4";                // world/global frame
#define FRAME_BODY "PX4/odom_local_ned";  // drone body frame

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
multi_drone_inspection::MissionStateStamped mission_state_msg;
mission_state_msg.state = PASSIVE;
auto seq_state = 0;
auto exploration_complete = false;
auto inspection_complete = false;

// home pose
auto home = eigen::Vector3f(0, 0, 5);
auto expected_pos = move(home);

//--------------------------------------------------------------------------------------------------
// ROS
//--------------------------------------------------------------------------------------------------

// publishers
ros::Publisher pub_mission_state;

// services
ros::ServiceClient client_arm;
ros::ServiceClient client_mode;
ros::ServiceClient client_land;

// time
ros::Time start_time;

// transform utilities
tf2_ros::Buffer tf_buffer;

// state variables
mavros_msgs::State state;

//--------------------------------------------------------------------------------------------------
// Bezier Spline
//--------------------------------------------------------------------------------------------------

BezierSpline spline;
auto spline_input_points =
    vector<Vector3f>{Vector3f(0.0, 0.0, 0.0),  Vector3f(3.0, 0.5, 1.0), Vector3f(-3.5, 1.5, 0.0),
                     Vector3f(-2.8, 1.0, 0.7), Vector3f(1.2, 2.2, 1.5), Vector3f(1.0, 3.0, 1.0)};
auto forwards = true;

//--------------------------------------------------------------------------------------------------
// Callback Functions
//--------------------------------------------------------------------------------------------------

auto state_cb(const mavros_msgs::State::ConstPtr& msg) -> void { state = *msg; }

//--------------------------------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------------------------------

auto main(int argc, char** argv) -> int {
    //----------------------------------------------------------------------------------------------
    // ROS initialisations
    ros::init(argc, argv, "mdi_mission_state_msg");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    // save start_time
    start_time = ros::Time::now();
    //----------------------------------------------------------------------------------------------
    // transform utilities
    auto tf_listener = mdi::utils::transform::TransformListener{};
    // tf2_ros::TransformListener tf_listener(tf_buffer);

    //----------------------------------------------------------------------------------------------
    // spline preprocessing
    auto spline_input_points = vector<Vector3f>{Vector3f(0.0, 0.0, 0.0),  Vector3f(3.0, 0.5, 1.0),
                                                Vector3f(-3.5, 1.5, 0.0), Vector3f(-2.8, 1.0, 0.7),
                                                Vector3f(1.2, 2.2, 1.5),  Vector3f(1.0, 3.0, 1.0)};
    for (auto& point : spline_input_points) {
        point *= 10;
    }

    spline = mdi::BezierSpline(spline_input_points);

    //----------------------------------------------------------------------------------------------
    // state subscriber
    sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    //----------------------------------------------------------------------------------------------
    // velocity publisher
    pub_mission_state<multi_drone_inspection::MissionStateStamped>("/mdi/state", 10);

    //----------------------------------------------------------------------------------------------
    // arm service client
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // mode service client
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // land service client
    client_land = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    //----------------------------------------------------------------------------------------------
    // wait for FCU connection
    while (ros::ok() && !state.connected) {
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
    auto error = eigen::Vecor3f(0, 0, 0);
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
        if (!drone_state.armed && (ros::Time::now() - previous_request_time > ros::Duration(5.0))) {
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
        error = tf_listener.transform_vec3(FRAME_BODY, FRAME_WORLD, expected_pos);
        auto delta_time = ros::Time::now() - start_time;

        switch (mission_state_msg.state) {
            case PASSIVE:
                mission_state_msg.state = HOME;
                break;
            case HOME:
                // 1. if within tolerance, go to EXPLORATION
                // 2. if it's the second time we're here, go to LAND
                expected_pos = home;
                if (error.norm() < TOLERANCE_DISTANCE) {
                    if (inspection_complete) {
                        mission_state_msg.state = LAND;
                    } else {
                        mission_state_msg.state = EXPLORATION;
                    }
                }
                break;
            case EXPLORATION:
                // when object is matched, go to INSPECTION
                // go through spline here, getting spline from BezierSpline getting input from RRT*
                expected_pos = spline.get_point_at_distance(delta_time.toSec() * TARGET_VELOCITY);
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
        geometry_msgs::Point p;
        p.x = expected_pos.x();
        p.y = expected_pos.y();
        p.z = expected_pos.z();
        mission_state_msg.Point = p;

        pub_mission_state.publish(mission_state_msg);
    }
}