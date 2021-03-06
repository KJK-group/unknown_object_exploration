#include <octomap_msgs/GetOctomap.h>
#include <ros/ros.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "uoe/octomap.hpp"
#include "ros/service.h"

using namespace std::string_literals;

std::unique_ptr<ros::ServiceClient> get_octomap_client;

auto call_get_octomap() -> uoe::Octomap* {
    auto request = octomap_msgs::GetOctomap::Request{};  // empty request
    auto response = octomap_msgs::GetOctomap::Response{};

    get_octomap_client->call(request, response);
    return response.map.data.size() == 0 ? nullptr : new uoe::Octomap{response.map};
}

auto main(int argc, char* argv[]) -> int {
    const auto node_name = "octomap_volume"s;
    ros::init(argc, argv, node_name);
    auto nh = ros::NodeHandle();

    const auto usage = [&] {
        std::cerr << "rosrun uoe " << node_name << " <octomap_server_topic> <filename.bt>" << '\n';
        std::cerr << "EXAMPLE\n"
                  << "rosrun uoe " << node_name << " /object_map_server octomap.bt" << '\n';
    };

    for (std::size_t i = 1; i < argc; ++i) {
        const auto arg = std::string(argv[i]);
        if (arg == "--help" || arg == "-h") {
            usage();
            exit(0);
        }
    }
#ifdef WRITE_BT_FILE
    if (argc < 3) {
        usage();
        std::exit(EXIT_FAILURE);
    }

    const auto output_bt_filepath = std::string(argv[2]);
#endif

    const auto octomap_server_topic = std::string(argv[1]);
    const auto object_map_client_topic_name = octomap_server_topic + "/octomap_binary"s;
    // if (! ros::service::exists(object_map_client_topic_name, true)) {
    //     std::exit(EXIT_FAILURE);
    // }
    ros::service::waitForService(object_map_client_topic_name);

    get_octomap_client = std::make_unique<ros::ServiceClient>(
        nh.serviceClient<octomap_msgs::GetOctomap>(object_map_client_topic_name));

    if (auto octomap_ptr = call_get_octomap()) {
#ifdef WRITE_BT_FILE
        const auto path = std::filesystem::path(output_bt_filepath);
        // if (path.is_directory()) {
        //     std::cerr << output_bt_filepath << " is a directory. Cannot write octomap to it."
        //               << '\n';
        //     std::exit(EXIT_FAILURE);
        // }

        std::cerr << "writing octomap to file: " << path << '\n';
        octomap_ptr->write(path);
#endif  // WRITE_BT_FILE

#ifdef COMPUTE_VOLUME_AND_WRITE_TO_STDERR
        // calculate the total volume of the received object map
        const auto volume_total = octomap_ptr->compute_total_volume_of_occupied_voxels();
        std::cerr << "total volume of occupied voxels in octomap: " << volume_total << " m3"
                  << '\n';
#endif  // COMPUTE_VOLUME_AND_WRITE_TO_STDERR

        delete octomap_ptr;
    }
#ifdef WRITE_BT_FILE
    else {
        std::cerr << "octomap received from " << object_map_client_topic_name
                  << " was EMPTY not writing to " << output_bt_filepath << '\n';
        exit(EXIT_FAILURE);
    }
#endif
    return 0;
}
