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
#include "mdi/utils/rviz/rviz.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h"

namespace mdi {

class Mission {
   public:
    Mission(ros::NodeHandle* nh, float velocity_target = 1, Eigen::Vector3f home = {0, 0, 5});
    enum state { PASSIVE, HOME, EXPLORATION, INSPECTION, LAND };
    static auto state_to_string(enum state s) -> std::string;
    auto add_interest_point(Eigen::Vector3f interest_point) -> void;

    auto get_drone_state() -> mavros_msgs::State;
    auto get_spline() -> BezierSpline;
    auto exploration_step() -> bool;
    auto drone_takeoff(float altitude = 0) -> bool;
    auto drone_land() -> bool;
    auto drone_set_mode(std::string mode = "OFFBOARD") -> bool;
    auto drone_arm() -> bool;
    auto publish() -> void;

    mdi_msgs::MissionStateStamped state;

   private:
    auto find_path(Eigen::Vector3f start, Eigen::Vector3f end) -> std::vector<Eigen::Vector3f>;
    auto state_cb(const mavros_msgs::State::ConstPtr& state) -> void;
    auto error_cb(const mdi_msgs::PointNormStamped::ConstPtr& error) -> void;

    vector<Eigen::Vector3f> interest_points;

    // ros::NodeHandle& node_handle;

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
    mdi_msgs::PointNormStamped position_error;

    // points
    Eigen::Vector3f home_position;
    Eigen::Vector3f expected_position;

    // time
    ros::Time start_time;
    ros::Duration delta_time;

    // path
    mdi::BezierSpline spline;
    int path_start_idx;
    int path_end_idx;

    float velocity_target;

    // state
    int seq_state;
    int step_count;
};
}  // namespace mdi
#endif  // _MDI_MISSION_MANAGER_HPP_