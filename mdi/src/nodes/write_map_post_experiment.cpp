#include <octomap_msgs/GetOctomap.h>
#include <ros/ros.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "mdi/octomap.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/MissionStateStamped.h"
#include "ros/service.h"

using namespace std::string_literals;

std::unique_ptr<ros::ServiceClient> get_octomap_client;

auto call_get_octomap() -> mdi::Octomap* {
    auto request = octomap_msgs::GetOctomap::Request{};  // empty request
    auto response = octomap_msgs::GetOctomap::Response{};

    get_octomap_client->call(request, response);
    return response.map.data.size() == 0 ? nullptr : new mdi::Octomap{response.map};
}

mdi_msgs::MissionStateStamped mission_state;
auto mission_cb(const mdi_msgs::MissionStateStamped::ConstPtr& state) -> void {
    mission_state = *state;
}

auto main(int argc, char* argv[]) -> int {
    const auto node_name = "get_octomap_from_server_and_write_to_bt_file"s;
    ros::init(argc, argv, node_name);
    auto nh = ros::NodeHandle();

    auto mission_sub = nh.subscribe<mdi_msgs::MissionStateStamped>(
        "/mdi/mission/state", mdi::utils::DEFAULT_QUEUE_SIZE, mission_cb);

    if (argc < 3) {
        std::exit(EXIT_FAILURE);
    }

    const auto octomap_server_topic = std::string(argv[1]);
    const auto output_bt_filepath = std::string(argv[2]);

    const auto object_map_client_topic_name = octomap_server_topic + "/octomap_binary"s;
    // if (! ros::service::exists(object_map_client_topic_name, true)) {
    //     std::exit(EXIT_FAILURE);
    // }
    ros::service::waitForService(object_map_client_topic_name);

    get_octomap_client = std::make_unique<ros::ServiceClient>(
        nh.serviceClient<octomap_msgs::GetOctomap>(object_map_client_topic_name));

    while (ros::ok() && mission_state.state != 4) {
        ros::spinOnce();
        ros::Rate(mdi::utils::DEFAULT_LOOP_RATE).sleep();
    }
    auto octomap_ptr = call_get_octomap();
    if (octomap_ptr) {
        const auto path = std::filesystem::path(output_bt_filepath);

        std::cerr << "writing octomap to file: " << path << '\n';
        octomap_ptr->write(path);

        delete octomap_ptr;
    } else {
        std::cerr << "octomap received from " << object_map_client_topic_name
                  << " was EMPTY not writing to " << output_bt_filepath << '\n';
        exit(EXIT_FAILURE);
    }

    return 0;
}
