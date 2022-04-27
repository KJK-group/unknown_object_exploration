#include "mdi/mission_manager.hpp"

namespace mdi {
MissionManager::MissionManager(ros::NodeHandle& nh, float velocity_target, Eigen::Vector3f home = {0, 0, 5})
    : node_handle(nh), velocity_target(velocity_target), home_position(std::move(home)) {
    // publishers
    pub_mission_state = node_handle.advertise<mdi_msgs::MissionStateStamped>("/mdi/state", utils::DEFAULT_QUEUE_SIZE);
    pub_visualise =
        node_handle.advertise<visualization_msgs::Marker>("/mdi/visualisation_marker", utils::DEFAULT_QUEUE_SIZE);

    // subscribers
    sub_drone_state = node_handle.subscribe<mavros_msgs::State>(
        "/mavros/state", utils::DEFAULT_QUEUE_SIZE,
        [&](const mavros_msgs::State::ConstPtr& state) { drone_state = *state; });
    sub_position_error = node_handle.subscribe<mdi_msgs::PointNormStamped>(
        "/mdi/error", utils::DEFAULT_QUEUE_SIZE,
        [&](const mdi_msgs::PointNormStamped::ConstPtr& error) { position_error = *error; });

    // services
    client_arm = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    client_land = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    client_land = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    // time
    start_time = ros::Time::now();
    delta_time = ros::Duration(0);
}

auto MissionManager::find_path(Eigen::Vector3f start, Eigen::Vector3f end) -> std::vector<Eigen::Vector3f> {
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
                             .arrow_head_width(0.05f)
                             .arrow_length(0.05f)
                             .arrow_width(0.05f)
                             .color({0, 1, 0, 1})
                             .build();
    arrow_msg_gen.header.frame_id = utils::FRAME_WORLD;

    // rrt.register_cb_for_event_on_new_node_created([&](const auto& parent, const auto& new_node) {
    //     // std::cout << GREEN << parent << "\n" << MAGENTA << new_node << RESET << std::endl;
    //     auto msg = arrow_msg_gen({parent, new_node});
    //     msg.color.r = 0.5;
    //     msg.color.g = 1;
    //     msg.color.b = 0.9;
    //     msg.color.a = 1;
    //     pub_visualize_rrt.publish(msg);
    // });

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
        pub_visualize_rrt.publish(arrow);
        ++i;
    }

    return path;
}

}  // namespace mdi