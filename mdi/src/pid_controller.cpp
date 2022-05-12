#include "mdi/pid_controller.hpp"

namespace mdi::control {

PIDController::PIDController(ros::NodeHandle& nh, ros::Rate& rate, PID gains_xy, PID gains_z, PID gains_yaw,
                             bool visualise)
    : rate(rate),
      gains_xy(gains_xy),
      gains_z(gains_z),
      gains_yaw(gains_yaw),
      error_position({0, 0, 0}),
      error_position_integral({0, 0, 0}),
      error_position_previous({0, 0, 0}),
      error_yaw(0),
      error_yaw_integral(0),
      error_yaw_previous(0),
      command_linear({0, 0, 0}),
      command_yaw(0),
      seq_command(0),
      seq_state(0),
      seq_visualise(0),
      should_visualise(visualise) {
    // state subscriber
    sub_mission_state = nh.subscribe<mdi_msgs::MissionStateStamped>(
        "/mdi/mission/state", mdi::utils::DEFAULT_QUEUE_SIZE, &PIDController::mission_state_cb, this);
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", mdi::utils::DEFAULT_QUEUE_SIZE,
                                                &PIDController::odom_cb, this);

    // std::cout << "Inside controller" << std::endl;
    // std::cout << "gains_xy: \n" << gains_xy.p << " " << gains_xy.i << " " << gains_xy.d << std::endl;
    // std::cout << "gains_z: \n" << gains_z.p << " " << gains_z.i << " " << gains_z.d << std::endl;
    // std::cout << "gains_yaw: \n" << gains_yaw.p << " " << gains_yaw.i << " " << gains_yaw.d << std::endl;
    // velocity publisher
    pub_velocity =
        nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", mdi::utils::DEFAULT_QUEUE_SIZE);
    // state publisher
    pub_state = nh.advertise<mdi_msgs::ControllerStateStamped>("/mdi/controller/state", mdi::utils::DEFAULT_QUEUE_SIZE);
    // visualisation publisher
    pub_visualisation =
        nh.advertise<visualization_msgs::Marker>("/mdi/visualisation/controller", mdi::utils::DEFAULT_QUEUE_SIZE);
}

/**
 * @brief takes a 3D position error `pos_error` and applies PID control, converting it to a geometry_msgs::TwistStamped
 * message with the respective 3D velocities.
 *
 * @param pos_error is the 3D euclidean error
 * @return geometry_msgs::TwistStamped PID control output in a geometry_msgs::TwistStamped message
 */
auto PIDController::compute_linear_velocities() -> void {
    // std::cout << "Inside compute_linear_velocities" << std::endl;
    // std::cout << "gains_xy: \n" << gains_xy.p << " " << gains_xy.i << " " << gains_xy.d << std::endl;
    // std::cout << "gains_z: \n" << gains_z.p << " " << gains_z.i << " " << gains_z.d << std::endl;
    // std::cout << "gains_yaw: \n" << gains_yaw.p << " " << gains_yaw.i << " " << gains_yaw.d << std::endl;
    error_position = mdi::utils::transform::geometry_mgs_point_to_vec(mission_state.target.position) -
                     mdi::utils::transform::geometry_mgs_point_to_vec(odom.pose.pose.position);
    // update integral error if the drone is in the air
    if (odom.pose.pose.position.z > mdi::utils::SMALL_DISTANCE_TOLERANCE) {
        error_position_integral += error_position;
    }
    // change in error since previous time step
    auto error_derivative = error_position - error_position_previous;

    // linear velocity controller output
    // apply seperate gains for xy and z
    auto x_vel =
        gains_xy.p * error_position.x() + gains_xy.i * error_position_integral.x() + gains_xy.d * error_derivative.x();
    auto y_vel =
        gains_xy.p * error_position.y() + gains_xy.i * error_position_integral.y() + gains_xy.d * error_derivative.y();
    auto z_vel =
        gains_z.p * error_position.z() + gains_z.i * error_position_integral.z() + gains_z.d * error_derivative.z();
    auto tmp_command_linear = Eigen::Vector3f{x_vel, y_vel, z_vel};
    // auto ep = gains_xy.p * error_position;
    // auto ei = gains_xy.i * error_position_integral;
    // auto ed = gains_xy.d * error_derivative;
    // auto tmp_command_linear = ep + ei + ed;
    // set class field used to publish
    command_linear = mdi::utils::eigen::clamp_vec3(tmp_command_linear, VELOCITY_MIN, VELOCITY_MAX);

    error_position_previous = error_position;
}

/**
 * @brief take a quaternion target `quat` and applies PID control, converting it to a yaw velocity, that will correct
 * the error
 *
 * @param quat is the target attitude for the drone
 * @return float yaw velocity
 */
auto PIDController::compute_yaw_velocity() -> void {
    // target attitude
    auto target_yaw = tf2::getYaw(mission_state.target.orientation);

    // attitude error
    auto yaw = utils::state::clamp_yaw(tf2::getYaw(drone_attitude) + M_PI / 2);
    error_yaw = utils::state::clamp_yaw(target_yaw - yaw);

    // update integral error if the drone is in the air
    if (odom.pose.pose.position.z > utils::SMALL_DISTANCE_TOLERANCE) {
        error_yaw_integral += error_yaw;
    }
    // change in error since previous time step
    auto error_yaw_derivative = error_yaw - error_yaw_previous;
    // pid output
    auto tmp_command_yaw =
        gains_yaw.p * error_yaw + gains_yaw.i * error_yaw_integral + gains_yaw.d * error_yaw_derivative;
    // set class field used to publish
    command_yaw = std::clamp(tmp_command_yaw, VELOCITY_MIN_YAW, VELOCITY_MAX_YAW);

    error_yaw_previous = error_yaw;
}
/**
 * @brief publishes all necessary controller information, such as position error, to the topic /mdi/controller/error
 *
 */
auto PIDController::publish() -> void {
    // velocity control message
    geometry_msgs::TwistStamped twist_msg{};

    twist_msg.header.seq = seq_command++;
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.header.frame_id = mdi::utils::FRAME_WORLD;

    twist_msg.twist.linear.x = command_linear.x();
    twist_msg.twist.linear.y = command_linear.y();
    twist_msg.twist.linear.z = command_linear.z();
    twist_msg.twist.angular.z = command_yaw;

    pub_velocity.publish(twist_msg);

    // state message
    mdi_msgs::ControllerStateStamped state_msg;
    state_msg.header.seq = seq_state++;
    state_msg.header.stamp = ros::Time::now();
    state_msg.header.frame_id = mdi::utils::FRAME_WORLD;

    state_msg.command = twist_msg.twist;

    state_msg.error.position.x = error_position.x();
    state_msg.error.position.y = error_position.y();
    state_msg.error.position.z = error_position.z();
    state_msg.error.position.norm = error_position.norm();
    state_msg.error.yaw = error_yaw;

    pub_state.publish(state_msg);
}
/**
 * @brief publishes visualisation_msg::Marker messages to the topic /mdi/visualisation/controller for visualisation in
 * RViz
 *
 */
auto PIDController::visualise() -> void {
    auto m = utils::rviz::sphere_msg_gen()(utils::transform::geometry_mgs_point_to_vec(odom.pose.pose.position) +
                                           error_position);
    m.header.frame_id = utils::FRAME_WORLD;

    m.color.a = 1;
    m.color.r = 1;
    m.color.g = 0;
    m.color.b = 0;

    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;

    // std::cout << mdi::utils::MAGENTA << "PUBLISHING" << RESET << std::endl;
    // std::cout << mdi::utils::ITALIC << mdi::utils::BOLD << "x: " << m.pose.position.x << "\ty: " << m.pose.position.y
    //           << "\tz: " << m.pose.position.z << std::endl;

    auto pos = utils::transform::geometry_mgs_point_to_vec(odom.pose.pose.position);
    pub_visualisation.publish(m);

    auto m2 = utils::rviz::arrow_msg_gen::Builder()
                  .arrow_head_width(0.05)
                  .arrow_width(0.025)
                  .arrow_length(0.05)
                  .color(0, 1, 0, 1)
                  .build()({pos, pos + command_linear});
    m2.header.frame_id = utils::FRAME_WORLD;
    m2.id = -1;
    pub_visualisation.publish(m2);
}

/**
 * @brief called when new message is published on the odometry topic /mavros/local_position/odom. sets class field
 * `odom` to the contents of the published message.
 *
 */
auto PIDController::odom_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void { odom = *msg; }
/**
 * @brief called when new message is published on the mdi state topic /mdi/mission/state. sets class field `state` to
 * the contents of the published message.
 *
 */
auto PIDController::mission_state_cb(const mdi_msgs::MissionStateStamped::ConstPtr& msg) -> void {
    mission_state = *msg;
}

/**
 * @brief runs the PID controller, looping with the given ros::Rate
 *
 */
auto PIDController::run() -> void {
    while (ros::ok()) {
        auto expected_position = Eigen::Vector3f(mission_state.target.position.x, mission_state.target.position.y,
                                                 mission_state.target.position.z);

        tf2::Quaternion drone_attitude_ned;
        tf2::fromMsg(odom.pose.pose.orientation, drone_attitude_ned);
        drone_attitude = drone_attitude_ned * tf2::Quaternion(std::sqrt(2) / 2, -std::sqrt(2) / 2, 0, 0);

        compute_linear_velocities();
        compute_yaw_velocity();

        if (mission_state.state != 4) {
            publish();
            if (should_visualise) {
                visualise();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}
}  // namespace mdi::control