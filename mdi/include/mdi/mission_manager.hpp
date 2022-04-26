#ifndef _MDI_MISSION_MANAGER_HPP_
#define _MDI_MISSION_MANAGER_HPP_

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include "mdi/bezier_spline.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h"
#include "ros/publisher.h"

namespace mdi {
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

class MissionManager {
   public:
    MissionManager(float velocity_target);

   private:
    mavros_msgs::State drone_state;
    nav_msgs::Odometry drone_odom;
    mdi_msgs::MissionStateStamped mission_state;
    mdi_msgs::PointNormStamped position_error;

    // publishers
    ros::Publisher pub_mission_state;

    // subscribers
    ros::Subscriber sub_drone_state;
    ros::Subscriber sub_position_error;

    // services
    ros::ServiceClient client_arm;
    ros::ServiceClient client_mode;
    ros::ServiceClient client_land;

    // points
    Eigen::Vector3f home_position;
    Eigen::Vector3f expected_position;

    // time
    ros::Time start_time;
    ros::Duration delta_time;

    mdi::BezierSpline spline;

    float velocity_target;
};
}  // namespace mdi
#endif  // _MDI_MISSION_MANAGER_HPP_