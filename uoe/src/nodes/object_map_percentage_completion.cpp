#include <octomap_msgs/GetOctomap.h>
#include <ros/ros.h>

#include <cmath>
#include <memory>
#include <string>

#include "uoe/octomap.hpp"
#include "uoe/utils/utils.hpp"
#include "uoe_msgs/ObjectMapCompleteness.h"
#include "ros/init.h"
#include "ros/param.h"
#include "ros/rate.h"

std::unique_ptr<ros::ServiceClient> get_octomap_client;

auto call_get_octomap() -> uoe::Octomap* {
    auto request = octomap_msgs::GetOctomap::Request{};  // empty request
    auto response = octomap_msgs::GetOctomap::Response{};

    get_octomap_client->call(request, response);
    return response.map.data.size() == 0 ? nullptr : new uoe::Octomap{response.map};
}

auto main(int argc, char* argv[]) -> int {
    std::string topic;
    if (argc > 1) topic = argv[1];

    using namespace std::string_literals;

    ros::init(argc, argv, "object_map_percentage_completion"s);
    auto nh = ros::NodeHandle();

    const auto object_map_client_topic_name = topic;
    get_octomap_client = std::make_unique<ros::ServiceClient>(
        nh.serviceClient<octomap_msgs::GetOctomap>(object_map_client_topic_name));

    auto object_map_completeness_pub = [&] {
        const auto topic_name = "/uoe/object_map_percentage_completion"s + topic;
        return nh.advertise<uoe_msgs::ObjectMapCompleteness>(topic_name,
                                                             uoe::utils::DEFAULT_QUEUE_SIZE);
    }();

    const double prescanned_object_map_total_volume = [] {
        double value = 0.0;
        const auto param = "/uoe/experiment/prescanned_object_map_total_volume"s;
        if (! ros::param::get(param, value)) {
            ROS_ERROR_STREAM("" << param << " not found on the parameter server."s);
            std::exit(EXIT_FAILURE);
        }
        return value;
    }();

    const auto publish_object_map_completeness = [&](const double volume) {
        auto msg = uoe_msgs::ObjectMapCompleteness{};
        msg.so_far = volume;
        msg.total = prescanned_object_map_total_volume;
        msg.percentage = msg.so_far / msg.total * 100.0;

        object_map_completeness_pub.publish(msg);
        // ros::spinOnce();
        // ros::Rate(uoe::utils::DEFAULT_LOOP_RATE).sleep();
    };

    while (ros::ok()) {
        if (auto octomap = call_get_octomap()) {
            ROS_INFO_STREAM("calculating object map completeness...");

            // calculate the total volume of the received object map
            const auto volume_total = octomap->compute_total_volume_of_occupied_voxels();
            std::cout << topic << " | " << volume_total << std::endl;
            publish_object_map_completeness(volume_total);

            delete octomap;
        } else {
            ROS_WARN_STREAM("no object map received by service on topic "
                            << object_map_client_topic_name);
        }

        ros::spinOnce();
        ros::Rate(uoe::utils::DEFAULT_LOOP_RATE).sleep();
    }

    return 0;
}
