#include "mdi/mission.hpp"

namespace mdi {
Mission::Mission(ros::NodeHandle* nh, float velocity_target, Eigen::Vector3f home)
    : seq_state(0),
      path_start_idx(0),
      path_end_idx(1),
      velocity_target(velocity_target),
      home_position(std::move(home)) {
    // publishers
    pub_mission_state = nh->advertise<mdi_msgs::MissionStateStamped>("/mdi/state", utils::DEFAULT_QUEUE_SIZE);
    pub_visualise = nh->advertise<visualization_msgs::Marker>("/mdi/visualisation_marker", utils::DEFAULT_QUEUE_SIZE);

    // subscribers
    sub_drone_state =
        nh->subscribe<mavros_msgs::State>("/mavros/state", utils::DEFAULT_QUEUE_SIZE, &Mission::state_cb, this);
    sub_position_error =
        nh->subscribe<mdi_msgs::PointNormStamped>("/mdi/error", utils::DEFAULT_QUEUE_SIZE, &Mission::error_cb, this);

    // services
    client_arm = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    client_mode = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    client_land = nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    client_land = nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    // time
    start_time = ros::Time::now();
    delta_time = ros::Duration(0);

    // state
    state.header.frame_id = mdi::utils::FRAME_WORLD;
    state.state = PASSIVE;
    state.target.position.x = home_position.x();
    state.target.position.y = home_position.y();
    state.target.position.z = home_position.z();

    // path
    interest_points.push_back(home_position);
}

auto Mission::state_cb(const mavros_msgs::State::ConstPtr& state) -> void { drone_state = *state; }
auto Mission::error_cb(const mdi_msgs::PointNormStamped::ConstPtr& error) -> void { position_error = *error; }

auto Mission::add_interest_point(Eigen::Vector3f interest_point) -> void { interest_points.push_back(interest_point); }
auto Mission::get_drone_state() -> mavros_msgs::State { return drone_state; }
auto Mission::get_spline() -> BezierSpline { return spline; }

/**
 * @brief requests px4 to takeoff through mavros
 *
 * @param altitude altitude to hover at
 * @return true if request is accepted
 * @return false if request is rejected or a takeoff procedure is in progress
 */
auto Mission::drone_takeoff(float altitude) -> bool {
    if (drone_state.mode != "AUTO.LAND") {
        mavros_msgs::CommandTOL takeoff_msg;
        takeoff_msg.request.altitude = (altitude == 0) ? home_position.z() : altitude;

        return (client_land.call(takeoff_msg)) ? true : false;
    } else {
        return false;
    }
}
/**
 * @brief requests px4 to land through mavros
 *
 * @return true if request is accepted
 * @return false if request is rejected
 */
auto Mission::drone_land() -> bool {
    if (drone_state.mode != "AUTO.LAND") {
        mavros_msgs::CommandTOL land_msg;
        land_msg.request.altitude = 0;

        return (client_land.call(land_msg)) ? true : false;
    } else {
        return false;
    }
}
auto Mission::drone_set_mode(std::string mode) -> bool {
    mavros_msgs::SetMode mode_msg{};
    mode_msg.request.custom_mode = mode;

    if (drone_state.mode != mode) {
        return (client_mode.call(mode_msg) && mode_msg.response.mode_sent) ? true : false;
    } else {
        return false;
    }
}
auto Mission::drone_arm() -> bool {
    mavros_msgs::CommandBool srv{};
    srv.request.value = true;

    if (! drone_state.armed) {
        return (client_arm.call(srv)) ? true : false;
    } else {
        return false;
    }
}

auto Mission::publish() -> void {
    state.header.seq = seq_state++;
    state.header.stamp = ros::Time::now();
}

auto Mission::find_path(Eigen::Vector3f start, Eigen::Vector3f end) -> std::vector<Eigen::Vector3f> {
    const auto goal_tolerance = 2;
    auto rrt = mdi::rrt::RRT::from_builder()
                   .probability_of_testing_full_path_from_new_node_to_goal(0)
                   .goal_bias(0.7)
                   .max_dist_goal_tolerance(goal_tolerance)
                   .start_and_goal_position(start, end)
                   .max_iterations(10000)
                   .step_size(2)
                   .build();

    auto arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                             .arrow_head_width(0.02f)
                             .arrow_length(0.02f)
                             .arrow_width(0.02f)
                             .color({0, 1, 0, 1})
                             .build();
    arrow_msg_gen.header.frame_id = utils::FRAME_WORLD;

    rrt.register_cb_for_event_on_new_node_created([&](const auto& parent, const auto& new_node) {
        // std::cout << GREEN << parent << "\n" << MAGENTA << new_node << RESET << std::endl;
        auto msg = arrow_msg_gen({parent, new_node});
        msg.color.r = 0.5;
        msg.color.g = 1;
        msg.color.b = 0.9;
        msg.color.a = 1;
        pub_visualise.publish(msg);
    });

    auto sphere_msg_gen = mdi::utils::rviz::sphere_msg_gen{};
    sphere_msg_gen.header.frame_id = utils::FRAME_WORLD;

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
    pub_visualise.publish(start_msg);
    // publish goal position
    auto goal_msg = sphere_msg_gen(end);
    goal_msg.color.r = 0;
    goal_msg.color.g = 1;
    goal_msg.color.b = 0;
    goal_msg.color.a = 1;
    goal_msg.scale.x = 0.2;
    goal_msg.scale.y = 0.2;
    goal_msg.scale.z = 0.2;
    pub_visualise.publish(goal_msg);

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
    pub_visualise.publish(msg);

    // ensure finding a path
    auto opt = rrt.run();
    while (! opt) {
        rrt.clear();
        opt = rrt.run();
    }

    const auto path = *opt;

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
        pub_visualise.publish(arrow);
        ++i;
    }

    return path;
}

auto Mission::exploration_step() -> bool {
    delta_time = ros::Time::now() - start_time;

    if (path_end_idx > interest_points.size()) {
        state.state = INSPECTION;
        return false;
    }

    auto distance = delta_time.toSec() * velocity_target;
    auto remaining_distance = spline.get_length() - distance;

    if (step_count == 0 || (remaining_distance < velocity_target * 2 && position_error.norm < velocity_target * 2)) {
        auto start = interest_points[path_start_idx++];
        auto end = interest_points[path_end_idx++];
        auto path = find_path(start, end);
        spline = mdi::BezierSpline(path);
        start_time = ros::Time::now();
        delta_time = ros::Time::now() - start_time;
    }

    std::cout << mdi::utils::GREEN << "delta_time: " << delta_time << mdi::utils::RESET << std::endl;

    // control the drone along the spline path
    distance = delta_time.toSec() * velocity_target;
    auto expected_pos = spline.get_point_at_distance(distance);

    state.target.position.x = expected_pos.x();
    state.target.position.y = expected_pos.y();
    state.target.position.z = expected_pos.z();

    publish();

    step_count++;
    return true;
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