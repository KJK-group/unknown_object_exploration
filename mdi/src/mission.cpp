#include "mdi/mission.hpp"

namespace mdi {
Mission::Mission(ros::NodeHandle& nh, ros::Rate& rate, types::vec3 target, float timeout,
                 float velocity_target, Eigen::Vector3f home, bool visualise)
    : inspection_complete_(false),
      exploration_complete_(false),
      takeoff_initiated_(false),
      home_trajectory_found_(false),
      step_count_(0),
      seq_point_(0),
      seq_state_(0),
      seq_vis_(0),
      waypoint_idx_(1),
      velocity_target_(velocity_target),
      rate_(rate),
      nh_(nh),
      home_position_(std::move(home)),
#ifdef PID_TEST
      expected_position_(home_position_),
#endif
#ifndef PID_TEST
      expected_position_(0, 0, 0),
#endif
      marker_scale_(0.1),
      should_visualise_(visualise),
      trajectory_(nh, rate, {{0, 0, 0}, home_position_}, visualise),
      target_(target),
      object_center_(target),
      timeout_(timeout),
      trajectory_start_time_(ros::Time::now()),
      timeout_start_time_(ros::Time::now()),
      trajectory_delta_time_(ros::Duration(0)),
      timeout_delta_time_(ros::Duration(0)),
      start_time_(ros::Time::now()),
      duration_(0) {
    // publishers
    pub_mission_state_ = nh.advertise<mdi_msgs::MissionStateStamped>("/mdi/mission/state",
                                                                     utils::DEFAULT_QUEUE_SIZE);
    pub_visualise_ = nh.advertise<visualization_msgs::MarkerArray>("/mdi/mission/visualisation",
                                                                   utils::DEFAULT_QUEUE_SIZE);
    pub_setpoint_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_local/local",
                                                             utils::DEFAULT_QUEUE_SIZE);

    // subscribers
    sub_drone_state_ = nh.subscribe<mavros_msgs::State>("/mavros/state", utils::DEFAULT_QUEUE_SIZE,
                                                        &Mission::mavros_state_cb_, this);
    sub_controller_state_ = nh.subscribe<mdi_msgs::ControllerStateStamped>(
        "/mdi/controller/state", utils::DEFAULT_QUEUE_SIZE, &Mission::controller_state_cb_, this);
    sub_odom_ = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", utils::DEFAULT_QUEUE_SIZE, &Mission::odom_cb_, this);
    sub_object_completion_ = nh.subscribe<mdi_msgs::ObjectMapCompleteness>(
        "/mdi/object_map_percentage_completion/object_voxel_map/octomap_binary",
        utils::DEFAULT_QUEUE_SIZE, &Mission::object_map_completion_cb_, this);

    // services
    client_arm_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    client_mode_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    client_land_ = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    client_takeoff_ = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    client_nbv_ = nh.serviceClient<mdi_msgs::NBV>("/mdi/rrt_service/nbv");
    client_rrt_ = nh.serviceClient<mdi_msgs::RrtFindPath>("/mdi/rrt_service/find_path");

    // state
    state_.header.frame_id = mdi::utils::FRAME_WORLD;
    state_.state = PASSIVE;

    // path
    interest_points_.push_back(home_position_);

    auto experiment_param = std::map<std::string, double>();
    ros::param::get("/mdi/experiment", experiment_param);
    if (experiment_param.empty()) {
        std::cerr << "experiment_param is empty" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    experiment_param_ = experiment_param;
}

auto Mission::mavros_state_cb_(const mavros_msgs::State::ConstPtr& state) -> void {
    drone_state_ = *state;
}
auto Mission::controller_state_cb_(const mdi_msgs::ControllerStateStamped::ConstPtr& error)
    -> void {
    controller_state_ = *error;
}
auto Mission::odom_cb_(const nav_msgs::Odometry::ConstPtr& odom) -> void { drone_odom_ = *odom; }

auto Mission::object_map_completion_cb_(const mdi_msgs::ObjectMapCompleteness::ConstPtr& object)
    -> void {
    object_map_ = *object;
}

auto Mission::compute_attitude_() -> void {
    auto pos = drone_odom_.pose.pose.position;

    // std::cout << "target:\n" << target_ << std::endl;
    auto diff_x = target_.x() - pos.x;
    auto diff_y = target_.y() - pos.y;
    auto expected_yaw = std::atan2(diff_y, diff_x);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, expected_yaw);
    expected_attitude_ = quat;
}

// auto Mission::add_interest_point(Eigen::Vector3f interest_point) -> void {
//     interest_points_.push_back(interest_point);
// }
auto Mission::get_drone_state() -> mavros_msgs::State { return drone_state_; }
auto Mission::get_trajectory() -> trajectory::CompoundTrajectory { return trajectory_; }

// auto Mission::set_state(enum state s) -> void {
//     state_.state = s;
//     publish_();
// }
auto Mission::get_state() -> enum state { return (enum state)state_.state; }

/**
 * @brief requests mavros to land at current position
 *
 * @return true if request is accepted
 * @return false if request is rejected
 */
auto Mission::drone_land() -> bool {
    ROS_INFO_STREAM("Sending landing request to mavros");
    auto success = false;

    state_.state = LAND;
    mavros_msgs::CommandTOL land_msg;

    while (ros::ok() && drone_state_.mode != "AUTO.LAND") {
        success = (client_land_.call(land_msg));
        publish_();
        ros::spinOnce();
        rate_.sleep();
    }

    return success;
}
auto Mission::drone_set_mode_(std::string mode) -> bool {
    ROS_INFO_STREAM("Sending OFFBOARD mode request to mavros");
    auto success = false;

    mavros_msgs::SetMode mode_msg{};
    mode_msg.request.custom_mode = mode;

    while (ros::ok() && drone_state_.mode != mode) {
        success = (client_mode_.call(mode_msg) && mode_msg.response.mode_sent);
        publish_();
        ros::spinOnce();
        rate_.sleep();
    }
    return success;
}
auto Mission::drone_arm() -> bool {
    ROS_INFO_STREAM("Sending arm request to mavros");
    auto success = false;

    mavros_msgs::CommandBool srv{};
    srv.request.value = true;

    while (ros::ok() && ! drone_state_.armed) {
        success = (client_arm_.call(srv));
        publish_();
        ros::spinOnce();
        rate_.sleep();
    }
    return success;
}

auto Mission::publish_() -> void {
    state_.header.seq = seq_state_++;
    state_.header.stamp = ros::Time::now();
    state_.target.position.x = expected_position_.x();
    state_.target.position.y = expected_position_.y();
    state_.target.position.z = expected_position_.z();

    state_.target.orientation.x = expected_attitude_.getX();
    state_.target.orientation.y = expected_attitude_.getY();
    state_.target.orientation.z = expected_attitude_.getZ();
    state_.target.orientation.w = expected_attitude_.getW();

    state_.closest_point.x = closest_position_.x();
    state_.closest_point.y = closest_position_.y();
    state_.closest_point.z = closest_position_.z();

    pub_mission_state_.publish(state_);

    if (should_visualise_) {
        visualise_();
    }
}

auto Mission::visualise_() -> void {
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.id = -1;
    m.header.frame_id = utils::FRAME_WORLD;
    m.header.seq = seq_vis_++;
    m.header.stamp = ros::Time::now();

    m.type = visualization_msgs::Marker::SPHERE;

    m.pose.position.x = target_.x();
    m.pose.position.y = target_.y();
    m.pose.position.z = target_.z();

    m.scale.x = marker_scale_ * 2;
    m.scale.y = marker_scale_ * 2;
    m.scale.z = marker_scale_ * 2;

    m.color.a = 1;
    m.color.r = 1;
    m.color.g = 1;
    m.color.b = 1;

    ma.markers.push_back(m);

    m.id = -2;
    m.pose.position = mdi::utils::transform::vec_to_geometry_msg_point(closest_position_);

    m.color.a = 1;
    m.color.r = 1;
    m.color.g = 0;
    m.color.b = 0;

    ma.markers.push_back(m);

    pub_visualise_.publish(ma);
}

auto Mission::find_path_(Eigen::Vector3f start, Eigen::Vector3f end)
    -> std::optional<std::vector<Eigen::Vector3f>> {
    ROS_INFO_STREAM("Requesting RRT path from "
                    << "[" << start.x() << "," << start.y() << "," << start.z() << "] to "
                    << "[" << end.x() << "," << end.y() << "," << end.z() << "]");
    auto rrt_msg = mdi_msgs::RrtFindPath{};

    auto drone_param = std::map<std::string, double>();
    ros::param::get("/mdi/drone", drone_param);
    if (drone_param.empty()) {
        std::cerr << "drone_param is empty" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    auto& drone = rrt_msg.request.drone_config;
    drone.width = drone_param["width"];
    drone.height = drone_param["height"];
    drone.depth = drone_param["depth"];

    auto rrt_param = std::map<std::string, float>();
    ros::param::get("/mdi/rrt/params", rrt_param);
    if (rrt_param.empty()) {
        std::cerr << "rrt_param is empty" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    auto& rrt = rrt_msg.request.rrt_config;

    rrt.probability_of_testing_full_path_from_new_node_to_goal =
        rrt_param["probability_of_testing_full_path_from_new_node_to_goal"];
    rrt.goal_bias = rrt_param["goal_bias"];
    rrt.goal_tolerance = rrt_param["goal_tolerance"];
    rrt.start.x = start.x();
    rrt.start.y = start.y();
    rrt.start.z = start.z();
    rrt.goal.x = end.x();
    rrt.goal.y = end.y();
    rrt.goal.z = end.z();
    rrt.max_iterations = static_cast<uint>(rrt_param["max_iterations"]);
    rrt.step_size = rrt_param["step_size"];

    if (client_rrt_.call(rrt_msg)) {
        auto& waypoints = rrt_msg.response.waypoints;
        auto path = std::vector<Eigen::Vector3f>();
        // std::cout << "before for loop" << std::endl;
        for (auto& wp : waypoints) {
            // std::cout << wp << std::endl;
            path.emplace_back(wp.x, wp.y, wp.z);
        }

        if (interest_points_.back() != path.back()) {
            interest_points_.push_back(path.back());
        }
        std::reverse(path.begin(), path.end());
        return {path};
    }

    return {};
}

auto Mission::find_nbv_path_(Eigen::Vector3f start) -> std::optional<std::vector<Eigen::Vector3f>> {
    ROS_INFO_STREAM("Requesting NBV path from "
                    << "[" << start.x() << "," << start.y() << "," << start.z() << "]");

    auto nbv_srv = mdi_msgs::NBV{};
    {
        auto drone_param = std::map<std::string, double>();
        ros::param::get("/mdi/drone", drone_param);
        if (drone_param.empty()) {
            std::cerr << "drone_param is empty" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        auto& drone = nbv_srv.request.drone_config;
        drone.width = drone_param["width"];
        drone.height = drone_param["height"];
        drone.depth = drone_param["depth"];
    }

    {
        auto nbv_param = std::map<std::string, double>();
        ros::param::get("/mdi/nbv", nbv_param);
        if (nbv_param.empty()) {
            std::cerr << "nbv_param is empty" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        auto& nbv = nbv_srv.request.nbv_config;
        nbv.gain_of_interest_threshold = nbv_param["information_threshold"];
        nbv.weight_free = nbv_param["free"];
        nbv.weight_occupied = nbv_param["occupied"];
        nbv.weight_unknown = nbv_param["unknown"];
        nbv.weight_not_visible = nbv_param["not_visible"];
        nbv.weight_distance_to_object = nbv_param["distance_to_object"];
        nbv.excluded_points_distance_tolerance = nbv_param["excluded_points_distance_tolerance"];
        for (const auto& ip : interest_points_) {
            nbv.excluded_points.emplace_back(mdi::utils::transform::vec_to_geometry_msg_point(ip));
        }

        auto rrt_param = std::map<std::string, float>();
        ros::param::get("/mdi/rrt/params", rrt_param);
        if (rrt_param.empty()) {
            std::cerr << "rrt_param is empty" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        auto& rrt = nbv_srv.request.rrt_config;
        rrt.probability_of_testing_full_path_from_new_node_to_goal =
            rrt_param["probability_of_testing_full_path_from_new_node_to_goal"];
        rrt.goal_bias = rrt_param["goal_bias"];
        rrt.goal_tolerance = rrt_param["goal_tolerance"];
        rrt.start.x = start.x();
        rrt.start.y = start.y();
        rrt.start.z = start.z();
        rrt.goal.x = object_center_.x();
        rrt.goal.y = object_center_.y();
        rrt.goal.z = object_center_.z();
        rrt.max_iterations = static_cast<uint>(nbv_param["max_iterations"]);
        rrt.step_size = rrt_param["step_size"];
    }

    {
        auto camera_param = std::map<std::string, float>();
        ros::param::get("/mdi/camera", camera_param);
        if (camera_param.empty()) {
            std::cerr << "camera_param is empty" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        auto& fov = nbv_srv.request.fov;
        fov.depth_range.max = camera_param["depth_max"];
        fov.depth_range.min = camera_param["depth_min"];
        fov.horizontal.angle = camera_param["horisontal_fov"];
        fov.vertical.angle = camera_param["vertical_fov"];
        fov.pitch.angle = camera_param["pitch"];
    }

    if (client_nbv_.call(nbv_srv)) {
        auto path = std::vector<Eigen::Vector3f>();
        auto& waypoints = nbv_srv.response.waypoints;
        // TODO: should check if list of waypoints is NOT empty
        for (auto& wp : waypoints) {
            path.emplace_back(wp.x, wp.y, wp.z);
        }

        if (interest_points_.back() != path.back()) {
            interest_points_.push_back(path.back());
        }
        return {path};
    }

    return {};
}

auto Mission::fit_trajectory_(std::vector<Eigen::Vector3f> path)
    -> std::optional<trajectory::CompoundTrajectory> {
    // std::cout << "found path" << std::endl;
    // for (auto& p : path) {
    //     std::cout << p << std::endl;
    // }
    if (path.size() > 1) {
        auto valid = false;
        for (int p = 1; p < path.size(); ++p) {
            if ((path[0] - path[1]).norm() > std::numeric_limits<float>::epsilon()) {
                valid = true;
            }
        }
        if (valid) {
            return trajectory::CompoundTrajectory(nh_, rate_, path, should_visualise_);
        }
    }
    return {};
}

/**
 * @brief updates the expected position to the expect distance travelled at this time with the given
 * velocity
 *
 * @return true if the end of the current trajectory is reached
 * @return false as long as the end hasn't been reached
 */
auto Mission::trajectory_step_(float vel, bool look_forwards) -> bool {
    auto end_reached = false;
    timeout_start_time_ = ros::Time::now();  // reset mission countdown

    // start trajectory from the beginning in case we're starting a new trajectory
    if (step_count_ == 0) {
        trajectory_start_time_ = ros::Time::now();
    }

    // the trajectory progress
    trajectory_delta_time_ = ros::Time::now() - trajectory_start_time_;
    const auto distance = trajectory_delta_time_.toSec() * vel;
    const auto remaining_distance = trajectory_.get_length() - distance;
    // std::cout << mdi::utils::GREEN << "distance: " << distance << mdi::utils::RESET << std::endl;
    // std::cout << mdi::utils::GREEN << "remaining_distance: " << remaining_distance
    //           << mdi::utils::RESET << std::endl;
    // std::cout << mdi::utils::GREEN << "ERRORNORM: " << controller_state_.error.position.norm
    //           << mdi::utils::RESET << std::endl;

    // end of trajectory has been reached if the remaining distance and position error is small
    if (remaining_distance < utils::SMALL_DISTANCE_TOLERANCE &&
        controller_state_.error.position.norm < utils::SMALL_DISTANCE_TOLERANCE * 2) {
        std::cout << "end reached!" << std::endl;
        // in this case the heading should be towards the center of the object
        target_ = object_center_;
        end_reached = true;
    } else {  // otherwise the next expected position is computed for the current time step
        expected_position_ = trajectory_.get_point_at_distance(distance);
        // here the heading should be towards the expected position
        if (look_forwards) {
            target_ = expected_position_;
        }

        end_reached = false;
    }

    compute_attitude_();
    // publish_();
    step_count_++;

    return end_reached;
}

auto Mission::set_home_trajectory_() -> bool {
    // std::cout << "Going to home position" << std::endl;
    const auto start = interest_points_.back();
    std::cout << "last interest point\n" << start << std::endl;
    for (auto& ip : interest_points_) {
        std::cout << "\n" << ip << std::endl;
    }
    if (auto path_opt = find_path_(start, home_position_)) {
        auto path = path_opt.value();

        std::cout << "RRT PATH:\n" << std::endl;
        for (auto& ip : path) {
            std::cout << ip << std::endl;
        }

        if (const auto trajectory_opt = fit_trajectory_(path)) {
            trajectory_ = *trajectory_opt;
            step_count_ = 0;
            ROS_INFO_STREAM("Path viable");
            return true;
        } else {
            ROS_INFO_STREAM("Path not viable");
        }
    }
    return false;
}

auto Mission::set_takeoff_trajectory_() -> void {
    const auto offset = 2.5f;
    const auto path = std::vector<Eigen::Vector3f>{
        {0, 0, 0},
        home_position_,
        {home_position_.x() - offset, home_position_.y(), home_position_.z() + offset / 2},
        {home_position_.x() - offset, home_position_.y() - offset, home_position_.z() + offset / 2},
        {home_position_.x(), home_position_.y() - offset, home_position_.z() + offset / 2},
        {home_position_.x() + offset, home_position_.y() - offset, home_position_.z() + offset / 2},
        {home_position_.x() + offset, home_position_.y(), home_position_.z() + offset / 2},
        {home_position_.x() + offset, home_position_.y() + offset, home_position_.z() + offset / 2},
        {home_position_.x(), home_position_.y() + offset, home_position_.z() + offset / 2},
        {home_position_.x() - offset, home_position_.y() + offset, home_position_.z() + offset / 2},
        {home_position_.x() - offset, home_position_.y(), home_position_.z() + offset / 2},
        {home_position_.x() - offset, home_position_.y() - offset, home_position_.z() + offset / 2},
        {home_position_.x(), home_position_.y() - offset, home_position_.z() + offset / 2},
        {home_position_.x() + offset, home_position_.y() - offset, home_position_.z() + offset / 2},
        {home_position_.x() + offset, home_position_.y(), home_position_.z() + offset / 2},
        home_position_};

    if (auto trajectory_opt = fit_trajectory_(path)) {
        trajectory_ = *trajectory_opt;
        step_count_ = 0;
        ROS_INFO_STREAM("Path viable");
    } else {
        ROS_INFO_STREAM("Path not viable");
    }

    target_ = home_position_;
}

auto Mission::set_nbv_trajectory_() -> void {
    if (auto path_opt = find_nbv_path_(interest_points_.back())) {
        if (auto trajectory_opt = fit_trajectory_(path_opt.value())) {
            trajectory_ = *trajectory_opt;
            step_count_ = 0;
            ROS_INFO_STREAM("Path viable");
        } else {
            ROS_INFO_STREAM("Path not viable");
        }
    }
}

auto Mission::set_test_trajectory_() -> void {
    ROS_INFO_STREAM(mdi::utils::GREEN << "Setting PID testing trajectory");
    // const Eigen::Vector3f x_offset = {1, 0, 0};
    // const Eigen::Vector3f y_offset = {0, 1, 0};
    // const Eigen::Vector3f z_offset = {0, 0, 1};

    // const auto pitch_angle_up =
    //     Eigen::Quaternionf{Eigen::AngleAxisf{M_PI_4, Eigen::Vector3f::UnitX()}};
    // const auto pitch_angle_down =
    //     Eigen::Quaternionf{Eigen::AngleAxisf{-M_PI_4, Eigen::Vector3f::UnitX()}};
    // const auto yaw_angle_left =
    //     Eigen::Quaternionf{Eigen::AngleAxisf{M_PI_4, Eigen::Vector3f::UnitZ()}};
    // const auto yaw_angle_right =
    //     Eigen::Quaternionf{Eigen::AngleAxisf{-M_PI_4, Eigen::Vector3f::UnitZ()}};

    // const std::vector<std::vector<Eigen::Quaternionf>> turns = {
    //     // {Eigen::AngleAxisf{-M_PI_2, Eigen::Vector3f::UnitZ()}, pitch_angle_up},
    //     {},
    //     {},
    //     {},
    //     {},
    //     {},
    //     {yaw_angle_left},
    //     {},
    //     {},
    //     {yaw_angle_right},
    //     {},
    //     {},
    //     {yaw_angle_left},
    //     {yaw_angle_right},
    //     {},
    //     {},
    //     {yaw_angle_left, yaw_angle_left},
    //     {},
    //     {},
    //     {yaw_angle_right, yaw_angle_right},
    //     {},
    //     {},
    //     {yaw_angle_left, yaw_angle_left},
    //     {yaw_angle_right, yaw_angle_right},
    //     {},
    //     {},
    //     {yaw_angle_left, yaw_angle_left, yaw_angle_left},
    //     {},
    //     {},
    //     {yaw_angle_left, yaw_angle_left, yaw_angle_right},
    //     {},
    //     {},
    //     {yaw_angle_right, yaw_angle_right, yaw_angle_right},
    //     {yaw_angle_left, yaw_angle_left, yaw_angle_left},
    //     {},
    //     {},
    //     {yaw_angle_right, yaw_angle_right, yaw_angle_right},
    //     {yaw_angle_right, yaw_angle_right, yaw_angle_right},
    //     {},
    //     {},
    //     {yaw_angle_left, yaw_angle_left, yaw_angle_left},
    //     {yaw_angle_left, yaw_angle_left, yaw_angle_left},
    //     {},
    //     {},
    //     {yaw_angle_left, yaw_angle_left},
    //     {yaw_angle_left, yaw_angle_left},
    //     {yaw_angle_left, yaw_angle_left},
    //     {yaw_angle_right, yaw_angle_right},
    //     {yaw_angle_right, yaw_angle_right},
    //     {yaw_angle_right, yaw_angle_right},
    //     {},
    //     {},
    //     {}};

    // auto rolling_position = home_position_;
    // auto previous_dir = x_offset;

    std::vector<Eigen::Vector3f> path;
    // // path.emplace_back(0, 0, 0);
    // // path.emplace_back(rolling_position.x(), rolling_position.y(), rolling_position.z() - 1);
    // path.push_back(rolling_position);
    // for (auto& qs : turns) {
    //     auto dir = previous_dir;
    //     // auto unit_q = Eigen::Quaternionf{1,0,0,0};
    //     for (auto& q : qs) {
    //         dir = q * dir;
    //     }

    //     previous_dir = dir;
    //     rolling_position += dir;
    //     path.push_back(rolling_position);
    //     rolling_position += dir;
    //     path.push_back(rolling_position);
    //     // path.push_back(rolling_position);
    // }
    // path.push_back(home_position_);
    // if (auto trajectory_opt = fit_trajectory_(path)) {
    //     trajectory_ = *trajectory_opt;
    //     step_count_ = 0;
    //     ROS_INFO_STREAM("Path viable");
    // } else {
    //     ROS_INFO_STREAM("Path not viable");
    // }

    auto up = Eigen::Vector3f{0, 0, 2};
    auto down = Eigen::Vector3f{0, 0, -2};
    auto left = Eigen::Vector3f{0, -1, 0};
    auto right = Eigen::Vector3f{0, 1, 0};
    auto backwards = Eigen::Vector3f{-1, 0, 0};
    auto forwards = Eigen::Vector3f{1, 0, 0};

    const auto sections = std::vector<std::vector<Eigen::Vector3f>>{{up, forwards},
                                                                    {down, forwards},
                                                                    {up, forwards},
                                                                    {forwards},
                                                                    {up, forwards},
                                                                    {down, down, down, forwards},
                                                                    {up, up, up, forwards},
                                                                    {down, down, forwards},
                                                                    {up, forwards},
                                                                    {down, forwards},
                                                                    {forwards},
                                                                    {up, forwards},
                                                                    {down, down, forwards}};

    path.clear();
    auto rolling_position = home_position_;
    path.push_back(rolling_position);
    for (auto& s : sections) {
        for (auto& d : s) {
            rolling_position += d;
            // std::cout << d << std::endl;
        }
        path.push_back(rolling_position);
        for (auto& d : s) {
            rolling_position += d;
            // std::cout << d << std::endl;
        }
        path.push_back(rolling_position);
    }
    path.push_back(home_position_);

    if (auto trajectory_opt = fit_trajectory_(path)) {
        trajectory_ = *trajectory_opt;
        step_count_ = 0;
        ROS_INFO_STREAM("Path viable");
    } else {
        ROS_INFO_STREAM("Path not viable");
    }

    for (auto& p : path) {
        std::cout << "\n" << p << std::endl;
    }
    interest_points_.push_back(rolling_position);
}

auto Mission::run_step() -> void {
    auto should_end = false;
    duration_ = start_time_ - ros::Time::now();
    timeout_delta_time_ = ros::Time::now() - timeout_start_time_;

    if (timeout_delta_time_ - time_since_last_iteration_ > ros::Duration(1)) {
        time_since_last_iteration_ = timeout_delta_time_;
        ROS_INFO_STREAM("Mission cooldown: " << std::setw(6) << timeout_ - timeout_delta_time_);
    }

    // write output time to file when finished
    auto percentage_threshold = experiment_param_["object_map_percentage_threshold"];
    // std::cout << mdi::utils::GREEN << "completion: " << object_map_.percentage << "/"
    //           << percentage_threshold << mdi::utils::RESET << std::endl;
    // std::cout << mdi::utils::MAGENTA
    //           << "STATE: " << state_to_string(static_cast<state>(state_.state)) <<
    //           mdi::utils::RESET
    //           << std::endl;
    if (timeout_delta_time_ > timeout_) {
        std::cout << "Mission timed out" << std::endl;
        should_end = true;
    } else if (object_map_.percentage > percentage_threshold) {
        std::cout << "Exploration percentage threshold reached: " << object_map_.percentage << "/"
                  << percentage_threshold << std::endl;

        auto f = std::ofstream("~/Desktop/experiment_stats.txt", std::ios_base::app);
        f << experiment_param_["id"] << ":\t" << duration_.toSec() << "\n";
        f.close();
        should_end = true;
    }

    switch (state_.state) {
        case PASSIVE:
            drone_set_mode_();
            drone_arm();
            state_.state = HOME;
            break;
        case HOME:
            if (inspection_complete_) {
                if (! home_trajectory_found_) {
                    home_trajectory_found_ = set_home_trajectory_();
                }

                if (trajectory_step_(velocity_target_) && home_trajectory_found_) {
                    // if (should_end) {
                    //     end();
                    //     break;
                    // }
                    state_.state = LAND;
                }
            } else {
                if (! takeoff_initiated_) {
#ifndef PID_TEST
                    set_takeoff_trajectory_();
#endif
#ifdef PID_TEST
                    set_test_trajectory_();
                    // std::cout << mdi::utils::MAGENTA << "PID_TEST1" << mdi::utils::RESET
                    //           << std::endl;
                    // std::cout << mdi::utils::MAGENTA << state_to_string((state)state_.state)
                    //           << mdi::utils::RESET << std::endl;
#endif
                    takeoff_initiated_ = true;
                }
#ifndef PID_TEST
                if (trajectory_step_(velocity_target_, false)) {
                    if (should_end) {
                        end();
                        break;
                    }
                    state_.state = EXPLORATION;
                }
#endif
#ifdef PID_TEST
                if (trajectory_step_(velocity_target_, true)) {
                    // state_.state = LAND;
                    // inspection_complete_ = true;
                    // std::cout << mdi::utils::MAGENTA << "PID_TEST" << mdi::utils::RESET
                    //           << std::endl;
                    // std::cout << mdi::utils::MAGENTA << state_to_string((state)state_.state)
                    //           << mdi::utils::RESET << std::endl;
                }
#endif
            }
            break;
        case EXPLORATION:
            if (trajectory_step_(velocity_target_)) {
                if (should_end) {
                    end();
                    break;
                }
                ROS_INFO_STREAM((timeout_ - timeout_delta_time_).toSec()
                                << " Finding new NBV interest point...");
                publish_();
                set_nbv_trajectory_();
                // ros::Duration(10).sleep();
            }
            break;
        case INSPECTION:
            end();
            break;
        case LAND:
            // std::cout << "LANDING?" << std::endl;
            if (drone_state_.mode != "AUTO.LAND") {
                drone_land();
            } else {
                ros::Duration(5).sleep();
            }
            break;
        default:
            ROS_WARN_STREAM("Unknown state");
            break;
    }
    closest_position_ = trajectory_.get_closest_point(
        mdi::utils::transform::geometry_mgs_point_to_vec(drone_odom_.pose.pose.position));
    publish_();
}

auto Mission::run() -> void {
    while (ros::ok()) {
        run_step();
        ros::spinOnce();
        rate_.sleep();
    }
}

auto Mission::end() -> void {
    std::cout << utils::BOLD << utils::ITALIC << utils::MAGENTA << "Ending mission" << utils::RESET
              << std::endl;
    inspection_complete_ = true;
    if (state_.state == LAND &&
        drone_odom_.pose.pose.position.z < mdi::utils::SMALL_DISTANCE_TOLERANCE) {
        ros::shutdown();
        return;
    } else {
        state_.state = HOME;
    }
}

auto Mission::state_to_string(enum state s) -> std::string {
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

}  // namespace mdi
