#ifndef _MDI_MISSION_MANAGER_HPP_
#define _MDI_MISSION_MANAGER_HPP_

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Dense>
#include <utility>

#include "mdi/bezier_spline.hpp"
#include "mdi/rrt/rrt.hpp"
#include "mdi/rrt/rrt_builder.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h
#include "mdi/rrt/rrt_builder.hpp"
#include "mdi/utils/rviz/rviz.hpp"

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
    MissionManager(ros::NodeHandle& nh, float velocity_target, Eigen::Vector3f home);

   private:
    auto find_path(Eigen::Vector3f start, Eigen::Vector3f end) -> std::vector<Eigen::Vector3f>;

    ros::NodeHandle& node_handle;

    // publishers
    ros::Publisher pub_mission_state;
    ros::Publisher pub_visualise;

    // subscribers
    ros::Subscriber sub_drone_state;
    ros::Subscriber sub_position_error;

    // services
    ros::ServiceClient client_arm;
    ros::ServiceClient client_mode;
    ros::ServiceClient client_takeoff;
    ros::ServiceClient client_land;

    // msg instances
    mavros_msgs::State drone_state;
    nav_msgs::Odometry drone_odom;
    mdi_msgs::MissionStateStamped mission_state;
    mdi_msgs::PointNormStamped position_error;

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