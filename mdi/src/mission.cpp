#include "mdi/mission.hpp"

namespace mdi {
Mission::Mission(ros::NodeHandle& nh, ros::Rate& rate, float velocity_target, Eigen::Vector3f home)
    : inspection_complete(false),
      exploration_complete(false),
      step_count(0),
      seq_point(0),
      seq_state(0),
      path_start_idx(0),
      path_end_idx(1),
      velocity_target(velocity_target),
      rate(rate),
      home_position(std::move(home)) {
    // publishers
    pub_mission_state = nh.advertise<mdi_msgs::MissionStateStamped>("/mdi/state", utils::DEFAULT_QUEUE_SIZE);
    pub_visualise = nh.advertise<visualization_msgs::Marker>("/mdi/visualisation_marker", utils::DEFAULT_QUEUE_SIZE);
    pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_local/local", utils::DEFAULT_QUEUE_SIZE);

    // subscribers
    sub_drone_state =
        nh.subscribe<mavros_msgs::State>("/mavros/state", utils::DEFAULT_QUEUE_SIZE, &Mission::state_cb, this);
    sub_position_error =
        nh.subscribe<mdi_msgs::PointNormStamped>("/mdi/error", utils::DEFAULT_QUEUE_SIZE, &Mission::error_cb, this);

    // services
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    client_land = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    client_takeoff = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

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
auto Mission::odom_cb(const nav_msgs::Odometry::ConstPtr& odom) -> void { drone_odom = *odom; }

auto Mission::add_interest_point(Eigen::Vector3f interest_point) -> void { interest_points.push_back(interest_point); }
auto Mission::get_drone_state() -> mavros_msgs::State { return drone_state; }
auto Mission::get_spline() -> BezierSpline { return spline; }

auto Mission::set_state(enum state s) -> void {
    state.state = s;
    publish();
}
auto Mission::get_state() -> enum state { return (enum state)state.state; }

/**
 * @brief requests px4 to takeoff through mavros
 *
 * @param altitude altitude to hover at
 * @return true if request is accepted
 * @return false if request is rejected or a takeoff procedure is in progress
 */
auto Mission::drone_takeoff(float altitude) -> bool {
    // if (drone_state.mode != "AUTO.TAKEOFF") {
    //     mavros_msgs::CommandTOL takeoff_msg;
    //     takeoff_msg.request.altitude = home_position.z();

    //     return (client_takeoff.call(takeoff_msg)) ? true : false;
    // } else {
    //     // std::cout << "Already in AUTO.LAND" << std::endl;
    //     return false;
    // }
    ros::Duration(2).sleep();
    while (ros::ok() && position_error.norm > utils::SMALL_DISTANCE_TOLERANCE) {
        expected_position = Eigen::Vector3f(0, 0, utils::DEFAULT_DISTANCE_TOLERANCE);
        publish();
        ros::spinOnce();
        rate.sleep();
    }

    auto altitude_reached = false;
    start_time = ros::Time::now();

    while (ros::ok() && ! altitude_reached) {
        delta_time = ros::Time::now() - start_time;
        auto altitude_progress = delta_time.toSec() * velocity_target;
        if (altitude_progress > altitude) {
            altitude_progress = altitude;
            altitude_reached = true;
        }
        expected_position = Eigen::Vector3f(0, 0, altitude_progress);
        publish();
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && position_error.norm > utils::DEFAULT_DISTANCE_TOLERANCE) {
        publish();
        ros::spinOnce();
        rate.sleep();
    }

    state.state = EXPLORATION;

    return altitude_reached;
}
/**
 * @brief requests px4 to land through mavros
 *
 * @return true if request is accepted
 * @return false if request is rejected
 */
auto Mission::drone_land() -> bool {
    auto previous_request_time = ros::Time(0);
    auto success = false;

    state.state = LAND;
    mavros_msgs::CommandTOL land_msg;

    while (ros::ok() && drone_state.mode != "AUTO.LAND") {
        success = (client_land.call(land_msg)) ? true : false;
        publish();
        ros::spinOnce();
        rate.sleep();
    }

    return success;
}
auto Mission::drone_set_mode(std::string mode) -> bool {
    auto previous_request_time = ros::Time(0);
    auto success = false;

    mavros_msgs::SetMode mode_msg{};
    mode_msg.request.custom_mode = mode;

    while (ros::ok() && drone_state.mode != mode) {
        success = (client_mode.call(mode_msg) && mode_msg.response.mode_sent) ? true : false;
        publish();
        ros::spinOnce();
        rate.sleep();
    }
    return success;
}
auto Mission::drone_arm() -> bool {
    auto previous_request_time = ros::Time(0);
    auto success = false;

    mavros_msgs::CommandBool srv{};
    srv.request.value = true;

    while (ros::ok() && ! drone_state.armed) {
        success = (client_arm.call(srv)) ? true : false;
        publish();
        ros::spinOnce();
        rate.sleep();
    }
    return success;
}

auto Mission::publish() -> void {
    state.header.seq = seq_state++;
    state.header.stamp = ros::Time::now();
    state.target.position.x = expected_position.x();
    state.target.position.y = expected_position.y();
    state.target.position.z = expected_position.z();
    // std::cout << "state.target.position.x: " << state.target.position.x << std::endl;
    // std::cout << "state.target.position.y: " << state.target.position.y << std::endl;
    // std::cout << "state.target.position.z: " << state.target.position.z << std::endl;
    pub_mission_state.publish(state);
}

auto Mission::find_path(Eigen::Vector3f start, Eigen::Vector3f end) -> std::vector<Eigen::Vector3f> {
    // std::cout << "Finding path from " << start << " to " << end << std::endl;
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
        ros::spinOnce();
        rate.sleep();
    }

    // std::cout << rrt << std::endl;

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
    if (step_count == 0) {
        // std::cout << "resetting time" << std::endl;
        path_start_idx = 0;
        path_end_idx = 1;
        start_time = ros::Time::now();
    }
    delta_time = ros::Time::now() - start_time;
    // std::cout << mdi::utils::GREEN << "Exploration step at: " << delta_time << mdi::utils::RESET << std::endl;
    // std::cout << "step_count: " << step_count << std::endl;

    if (path_end_idx > interest_points.size()) {
        // std::cout << path_end_idx << std::endl;
        // std::cout << interest_points.size() << std::endl;
        // state.state = INSPECTION;
        ros::Duration(1).sleep();
        return false;
    }

    // std::cout << "Interest points:" << std::endl;
    // for (auto& ip : interest_points) {
    //     std::cout << ip << std::endl;
    // }

    // std::cout << "start_idx: " << path_start_idx << std::endl;
    // std::cout << "end_idx:   " << path_end_idx << std::endl;

    auto distance = delta_time.toSec() * velocity_target;
    auto remaining_distance = spline.get_length() - distance;
    // std::cout << mdi::utils::MAGENTA << "distance: " << distance << mdi::utils::RESET << std::endl;
    // std::cout << mdi::utils::MAGENTA << "remaining_distance: " << remaining_distance << mdi::utils::RESET <<
    // std::endl;

    // std::cout << "before if" << std::endl;
    if (step_count == 0 || remaining_distance < velocity_target && position_error.norm < velocity_target) {
        // std::cout << "inside if" << std::endl;
        auto start = interest_points[path_start_idx++];
        auto end = interest_points[path_end_idx++];
        auto path = find_path(start, end);

        // for (auto& p : path) {
        //     // std::cout << p << std::endl;
        // }

        spline = mdi::BezierSpline(path);
        start_time = ros::Time::now();
        delta_time = ros::Time::now() - start_time;
    }
    // std::cout << "after if" << std::endl;

    // std::cout << mdi::utils::GREEN << "delta_time: " << delta_time << mdi::utils::RESET << std::endl;

    // control the drone along the spline path
    distance = delta_time.toSec() * velocity_target;
    expected_position = spline.get_point_at_distance(distance);

    publish();

    step_count++;
    return true;
}

auto Mission::go_home() -> void {
    auto start = interest_points[path_end_idx];
    auto path = find_path(start, home_position);
    spline = mdi::BezierSpline(path);
}

auto Mission::run_step() -> void {
    switch (state.state) {
        case PASSIVE:
            drone_set_mode();
            drone_arm();
            set_state(HOME);
            break;
        case HOME:
            if (inspection_complete) {
                go_home();
                set_state(LAND);
            } else {
                drone_takeoff();
            }
            break;
        case EXPLORATION:
            if (! exploration_step()) {
                std::cout << "waiting for new interest point..." << std::endl;
            }
            break;
        case INSPECTION:
            inspection_complete = true;
            set_state(HOME);
            break;
        case LAND:
            drone_land();
            break;
        default:
            ROS_WARN_STREAM("Unknown state");
            break;
    }
}

auto Mission::run() -> void {
    while (ros::ok()) {
        run_step();
        ros::spinOnce();
        rate.sleep();
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