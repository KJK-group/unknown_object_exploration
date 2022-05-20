#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Dense>
#include <limits>
#include <optional>
#include <utility>

#include "common_types.hpp"
#include "mdi/compound_trajectory.hpp"
#include "mdi/rrt/rrt.hpp"
#include "mdi/rrt/rrt_builder.hpp"
#include "mdi/utils/rviz.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/ControllerStateStamped.h"
#include "mdi_msgs/MissionStateStamped.h"
#include "mdi_msgs/PointNormStamped.h"
#include "mdi_msgs/RrtFindPath.h"
#include "nav_msgs/Odometry.h"

namespace mdi {
constexpr auto INITIAL_ALTITUDE = 5;
class Mission {
   public:
    Mission(ros::NodeHandle& nh, ros::Rate& rate, types::vec2 target, float velocity_target = 1,
            Eigen::Vector3f home = {0, 0, INITIAL_ALTITUDE}, bool visualise = false);
    enum state { PASSIVE, HOME, EXPLORATION, INSPECTION, LAND };
    static auto state_to_string(enum state s) -> std::string;
    auto add_interest_point(Eigen::Vector3f interest_point) -> void;

    auto get_drone_state() -> mavros_msgs::State;
    auto get_trajectory() -> trajectory::CompoundTrajectory;
    auto drone_takeoff(float altitude = INITIAL_ALTITUDE) -> bool;
    auto drone_land() -> bool;
    auto drone_arm() -> bool;
    auto run() -> void;
    auto run_step() -> void;
    auto end() -> void;

    auto set_state(state state) -> void;
    auto get_state() -> state;

   private:
    auto find_path_(Eigen::Vector3f start, Eigen::Vector3f end) -> std::vector<Eigen::Vector3f>;
    auto fit_trajectory_(std::vector<Eigen::Vector3f> path)
        -> std::optional<trajectory::CompoundTrajectory>;
    auto drone_set_mode_(std::string mode = "OFFBOARD") -> bool;

    auto go_home_() -> void;
    auto trajectory_step_() -> bool;
    auto explore_() -> bool;
    auto exploration_step_() -> bool;
    auto publish_() -> void;

    auto mavros_state_cb_(const mavros_msgs::State::ConstPtr& state) -> void;
    auto controller_state_cb_(const mdi_msgs::ControllerStateStamped::ConstPtr& state) -> void;
    auto odom_cb_(const nav_msgs::Odometry::ConstPtr& odom) -> void;

    auto compute_attitude_() -> void;
    auto visualise_() -> void;

    ros::NodeHandle& nh_;
    ros::Rate& rate_;
    mdi_msgs::MissionStateStamped state_;

    std::vector<Eigen::Vector3f> interest_points_;
    std::vector<Eigen::Vector2f> target_points_;

    // publishers
    ros::Publisher pub_mission_state_;
    ros::Publisher pub_visualise_;
    ros::Publisher pub_setpoint_;

    // subscribers
    ros::Subscriber sub_drone_state_;
    ros::Subscriber sub_controller_state_;
    ros::Subscriber sub_odom_;

    // services
    ros::ServiceClient client_arm_;
    ros::ServiceClient client_mode_;
    ros::ServiceClient client_takeoff_;
    ros::ServiceClient client_land_;
    ros::ServiceClient client_rrt_;

    // msg instances
    mavros_msgs::State drone_state_;
    nav_msgs::Odometry drone_odom_;
    mdi_msgs::ControllerStateStamped controller_state_;

    // points
    Eigen::Vector3f home_position_;
    Eigen::Vector3f expected_position_;
    Eigen::Vector2f target_;
    Eigen::Vector2f object_center_;
    tf2::Quaternion expected_attitude_;

    // time
    ros::Time start_time_;
    ros::Time timeout_start_time_;
    ros::Duration delta_time_;
    ros::Duration timeout_delta_time_;
    ros::Duration timeout_;

    // path
    trajectory::CompoundTrajectory trajectory_;
    int waypoint_idx_;

    float velocity_target_;

    // state
    int seq_state_;
    int seq_point_;
    int seq_vis_;
    int step_count_;
    bool inspection_complete_;
    bool exploration_complete_;

    // visualisation
    float marker_scale_;
    bool should_visualise_;
};
}  // namespace mdi
