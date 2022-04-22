#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

auto foo(const nav_msgs::Odometry::ConstPtr& msg) -> void {}

auto main(int argc, char* argv[]) -> int {
    const auto name_of_node = "";
    ros::init(argc, argv, name_of_node);
    auto nh = ros::NodeHandle();
    auto rate = [] {
        const auto frequency = 10;  // Hz
        return ros::Rate(frequency);
    }();

    auto publisher = [&nh] {
        const auto topic = "/visualisation_marker";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::Marker>(topic, queue_size);
    }();

    auto subsciber = [&nh] {
        const auto topic = "/mavros/local_position/odom";
        const auto queue_size = 10;
        const auto cb = [](const nav_msgs::Odometry::ConstPtr& msg) -> void {};
        // this need c++ 20 :(
        return nh.subscribe<nav_msgs::Odometry>(topic, queue_size, cb);
    }();

    auto service = [&nh] {
        const auto handler = [](ServiceMsg::Request& request, ServiceMsg::Response& response) {
            // modify response to send data back
            return true;
        };
        const auto url = "service";
        return nh.advertiseService<ServiceMsg>(url, handler);
    }();

    auto service_client = [&nh] {
        const auto url = "/mavros/set_mode";
        return nh.serviceClient<mavros_msgs::SetMode>(url);
    }();

    // action ------------------------------------------------------------------

    auto action_server = [&nh] {
        return class {
           public:
           private:
            ac
        };
    }();

    auto action_client = [] {
        const auto url = "action_server";
        const auto client_spin_its_own_thread = true;
        // true causes the client to spin its own thread
        return actionlib::SimpleActionClient<class ActionSpec>(url, client_spin_its_own_thread);
    }();

    // use if publisher
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    // use if subscriber/service
    ros::spin();

    return 0;
}
