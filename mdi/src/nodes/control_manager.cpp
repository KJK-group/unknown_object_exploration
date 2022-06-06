#include <ros/ros.h>

#include "mdi/pid_controller.hpp"

auto main(int argc, char** argv) -> int {
    // ros
    ros::init(argc, argv, "mdi_pid_controller");
    auto nh = ros::NodeHandle();
    ros::Rate rate(mdi::utils::DEFAULT_LOOP_RATE * 6);

    mdi::control::PID gains_xy{};
    mdi::control::PID gains_yaw{};
    mdi::control::PID gains_z{};

    if (argc > 1) gains_xy.p = std::stof(argv[1]);
    if (argc > 2) gains_xy.i = std::stof(argv[2]);
    if (argc > 3) gains_xy.d = std::stof(argv[3]);
    if (argc > 4) gains_yaw.p = std::stof(argv[4]);
    if (argc > 5) gains_yaw.i = std::stof(argv[5]);
    if (argc > 6) gains_yaw.d = std::stof(argv[6]);
    if (argc > 7) gains_z.p = std::stof(argv[7]);
    if (argc > 8) gains_z.i = std::stof(argv[8]);
    if (argc > 9) gains_z.d = std::stof(argv[9]);

    if (argc <= 7) {
        gains_z = gains_xy;
    }

    // std::cout << "Inside node" << std::endl;
    // std::cout << "gains_xy: \n" << gains_xy.p << " " << gains_xy.i << " " << gains_xy.d <<
    // std::endl; std::cout << "gains_z: \n" << gains_z.p << " " << gains_z.i << " " << gains_z.d <<
    // std::endl; std::cout << "gains_yaw: \n" << gains_yaw.p << " " << gains_yaw.i << " " <<
    // gains_yaw.d << std::endl;

    // pid controller
    auto controller = mdi::control::PIDController(nh, rate, gains_xy, gains_z, gains_yaw, true);
    controller.run();

    return 0;
}