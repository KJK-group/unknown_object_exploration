#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <eigen3/Eigen/Dense>

#include "boost/format.hpp"
#include "multi_drone_inspection/MissionStateStamped.h"
#include "multi_drone_inspection/bezier_spline.hpp"

#include "utils/transformlistener.hpp"

#define TOLERANCE_DIST 0.1

enum state {PASSIVE, HOME, EXPLORATION, INSPECTION, LAND };
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
multi_drone_inspection::MissionStateStamped mission_state;
// home pose
auto home eigen::Vector3f(0,0,5);

//--------------------------------------------------------------------------------------------------
// ROS
//--------------------------------------------------------------------------------------------------

// publishers
ros::Publisher pub_mission_state;

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
    ros::init(argc, argv, "mdi_mission_state");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    // save start_time
    start_time = ros::Time::now();
    //----------------------------------------------------------------------------------------------
    // transform utilities
    auto tf_listener = utils::transform::TransformListener{};
    //tf2_ros::TransformListener tf_listener(tf_buffer);

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
    // wait for FCU connection
    while (ros::ok() && !state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    //----------------------------------------------------------------------------------------------
    // control loop
    while (ros::ok()) {
        
        switch (mission_state) {
            case PASSIVE:
                mission_state = HOME;
                break;
            case HOME:
                // 1. if within tolerance, go to EXPLORATION
                // 2. if it's the second time we're here, go to LAND
                break;
            case EXPLORATION:
                // when object is matched, go to INSPECITON
                break;
            case INSPECTION:
                // when object paths are completed, go to HOME
                break;
            case LAND:
                break;
            default:
                ROS_WARN_STREAM("Unknown state");
                break;
        }
    }
}