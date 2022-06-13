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
#include <unordered_map>
#include <utility>

#include "common_types.hpp"
#include "uoe/compound_trajectory.hpp"
#include "uoe/rrt/rrt.hpp"
#include "uoe/rrt/rrt_builder.hpp"
#include "uoe/utils/rviz.hpp"
#include "uoe/utils/transform.hpp"
#include "uoe/utils/utils.hpp"
#include "uoe_msgs/ControllerStateStamped.h"
#include "uoe_msgs/MissionStateStamped.h"
#include "uoe_msgs/NBV.h"
#include "uoe_msgs/ObjectMapCompleteness.h"
#include "uoe_msgs/PointNormStamped.h"
#include "uoe_msgs/RrtFindPath.h"
#include "nav_msgs/Odometry.h"

namespace uoe {
constexpr auto INITIAL_ALTITUDE = 5;
constexpr auto DEFAULT_TIMEOUT = 120;
class Mission {
   public:
    Mission(ros::NodeHandle& nh, ros::Rate& rate, types::vec3 target,
            float timeout = DEFAULT_TIMEOUT, float velocity_target = 1,
            Eigen::Vector3f home = {0, 0, INITIAL_ALTITUDE}, bool visualise = false);
    enum state { PASSIVE, HOME, EXPLORATION, INSPECTION, LAND };
    static auto state_to_string(enum state s) -> std::string;

    auto get_drone_state() -> mavros_msgs::State;
    auto get_trajectory() -> trajectory::CompoundTrajectory;
    auto run() -> void;
    auto run_step() -> void;
    auto end() -> void;

    auto get_state() -> state;

   private:
    auto find_path_(Eigen::Vector3f start, Eigen::Vector3f end)
        -> std::optional<std::vector<Eigen::Vector3f>>;
    auto find_nbv_path_(Eigen::Vector3f start) -> std::optional<std::vector<Eigen::Vector3f>>;
    auto fit_trajectory_(std::vector<Eigen::Vector3f> path)
        -> std::optional<trajectory::CompoundTrajectory>;
    auto drone_set_mode_(std::string mode = "OFFBOARD") -> bool;
    // auto set_state(state state) -> void;

    // auto drone_takeoff(float altitude = INITIAL_ALTITUDE) -> bool;
    auto drone_land() -> bool;
    auto drone_arm() -> bool;

    auto set_home_trajectory_() -> bool;
    auto set_takeoff_trajectory_() -> void;
    auto set_nbv_trajectory_() -> void;
    auto set_test_trajectory_() -> void;

    auto trajectory_step_(float vel, bool look_forwards = true) -> bool;
    auto publish_() -> void;

    auto mavros_state_cb_(const mavros_msgs::State::ConstPtr& state) -> void;
    auto controller_state_cb_(const uoe_msgs::ControllerStateStamped::ConstPtr& state) -> void;
    auto odom_cb_(const nav_msgs::Odometry::ConstPtr& odom) -> void;
    auto object_map_completion_cb_(const uoe_msgs::ObjectMapCompleteness::ConstPtr& object) -> void;

    auto compute_attitude_() -> void;
    auto visualise_() -> void;

    ros::NodeHandle& nh_;
    ros::Rate& rate_;
    uoe_msgs::MissionStateStamped state_;

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
    ros::Subscriber sub_object_completion_;

    // services
    ros::ServiceClient client_arm_;
    ros::ServiceClient client_mode_;
    ros::ServiceClient client_takeoff_;
    ros::ServiceClient client_land_;
    ros::ServiceClient client_nbv_;
    ros::ServiceClient client_rrt_;

    // msg instances
    mavros_msgs::State drone_state_;
    nav_msgs::Odometry drone_odom_;
    uoe_msgs::ControllerStateStamped controller_state_;
    uoe_msgs::ObjectMapCompleteness object_map_;

    // points
    Eigen::Vector3f home_position_;
    Eigen::Vector3f expected_position_;
    Eigen::Vector3f closest_position_;
    Eigen::Vector3f target_;
    Eigen::Vector3f object_center_;
    tf2::Quaternion expected_attitude_;

    // time
    ros::Time start_time_;
    ros::Time trajectory_start_time_;
    ros::Time timeout_start_time_;
    ros::Duration duration_;
    ros::Duration trajectory_delta_time_;
    ros::Duration timeout_delta_time_;
    ros::Duration time_since_last_iteration_;
    ros::Duration timeout_;

    // path
    trajectory::CompoundTrajectory trajectory_;
    int waypoint_idx_;

    float velocity_target_;
    // double percentage_threshold_;

    // state
    std::map<std::string, double> experiment_param_;
    int seq_state_;
    int seq_point_;
    int seq_vis_;
    int step_count_;
    bool inspection_complete_;
    bool exploration_complete_;
    bool takeoff_initiated_;
    bool home_trajectory_found_;

    // visualisation
    float marker_scale_;
    bool should_visualise_;
};
}  // namespace uoe
