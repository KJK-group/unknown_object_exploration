#pragma once

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include "geometry_msgs/TwistStamped.h"
#include "uoe/utils/eigen.hpp"
#include "uoe/utils/rviz.hpp"
#include "uoe/utils/state.hpp"
#include "uoe/utils/transform.hpp"
#include "uoe/utils/utils.hpp"
#include "uoe_msgs/ControllerStateStamped.h"
#include "uoe_msgs/MissionStateStamped.h"
#include "uoe_msgs/PointNormStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2/utils.h"

namespace uoe::control {

constexpr auto VELOCITY_MAX = 5.f;
constexpr auto VELOCITY_MIN = -VELOCITY_MAX;
constexpr auto VELOCITY_MAX_YAW = 1.f;
constexpr auto VELOCITY_MIN_YAW = -VELOCITY_MAX_YAW;

struct PID {
    float p, i, d;
};

class PIDController {
   public:
    PIDController(ros::NodeHandle& nh, ros::Rate& rate, PID gains_xy, PID gains_z, PID gains_yaw,
                  bool visualise = false);
    auto run() -> void;

   private:
    ros::Rate rate;

    auto compute_linear_velocities() -> void;
    auto compute_yaw_velocity() -> void;
    auto publish() -> void;
    auto visualise() -> void;

    auto odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void;
    auto mission_state_cb(const uoe_msgs::MissionStateStamped::ConstPtr& msg) -> void;

    // controller gains
    PID gains_xy;
    PID gains_z;
    PID gains_yaw;

    // ros
    ros::Publisher pub_velocity;
    ros::Publisher pub_state;
    ros::Publisher pub_visualisation;
    ros::Subscriber sub_mission_state;
    ros::Subscriber sub_odom;

    // cb variables
    uoe_msgs::MissionStateStamped mission_state;
    nav_msgs::Odometry odom;

    // state
    tf2::Quaternion drone_attitude;
    uoe_msgs::ControllerStateStamped state;

    // errors
    Eigen::Vector3f error_position;
    Eigen::Vector3f error_position_integral;
    Eigen::Vector3f error_position_previous;
    float error_yaw;
    float error_yaw_integral;
    float error_yaw_previous;

    // commands
    Eigen::Vector3f command_linear;
    float command_yaw;

    // sequence counters
    int seq_command;
    int seq_state;
    int seq_visualise;

    // visualisation
    bool should_visualise;
};
}  // namespace uoe::control
