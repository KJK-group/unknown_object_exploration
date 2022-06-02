#include <octomap_msgs/GetOctomap.h>
#include <ros/ros.h>

#include <cmath>
#include <memory>
#include <string>

#include "mdi/octomap.hpp"
#include "mdi_msgs/ObjectMapCompleteness.h"
#include "ros/init.h"
#include "ros/param.h"
#include "ros/rate.h"

std::unique_ptr<ros::ServiceClient> get_octomap_client;

auto call_get_octomap() -> mdi::Octomap* {
    auto request = octomap_msgs::GetOctomap::Request{};  // empty request
    auto response = octomap_msgs::GetOctomap::Response{};

    get_octomap_client->call(request, response);
    return response.map.data.size() == 0 ? nullptr : new mdi::Octomap{response.map};
}

auto main(int argc, char* argv[]) -> int {
    using namespace std::string_literals;

    ros::init(argc, argv, "object_map_percentage_completion"s);
    auto nh = ros::NodeHandle();

    const auto object_map_client_topic_name = "/object_voxel_map/octomap_binary"s;
    get_octomap_client = std::make_unique<ros::ServiceClient>(
        nh.serviceClient<octomap_msgs::GetOctomap>(object_map_client_topic_name));

    auto object_map_completeness_pub = [&] {
        const auto topic_name = "/mdi/object_map_percentage_completion"s;
        return nh.advertise<mdi_msgs::ObjectMapCompleteness>(topic_name, 10);
    }();

    const double prescanned_object_map_total_volume = [] {
        double value = 0.0;
        const auto param = "/mdi/prescanned_object_map_total_volume"s;
        if (! ros::param::get(param, value)) {
            ROS_ERROR_STREAM("" << param << " not found on the parameter server."s);
            std::exit(EXIT_FAILURE);
        }
        return value;
    }();

    const auto publish_object_map_completeness = [&](const double volume) {
        auto msg = mdi_msgs::ObjectMapCompleteness{};
        msg.so_far = volume;
        msg.total = prescanned_object_map_total_volume;
        msg.percentage = msg.so_far / msg.total * 100.0;

        object_map_completeness_pub.publish(msg);

        ros::spinOnce();
        ros::Rate(10).sleep();
    };

    while (ros::ok()) {
        if (auto octomap = call_get_octomap()) {
            ROS_INFO_STREAM("calculating object map completeness...");

            // calculate the total volume of the received object map
            const auto volume_total = octomap->compute_total_volume_of_occupied_voxels();
            publish_object_map_completeness(volume_total);

            delete octomap;
        } else {
            ROS_WARN_STREAM("no object map received by service on topic "
                            << object_map_client_topic_name);
        }

        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    return 0;
}
