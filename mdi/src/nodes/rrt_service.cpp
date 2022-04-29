#include <octomap/OcTree.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <iterator>
#include <memory>

#include "mdi/common_headers.hpp"
#include "mdi/octomap.hpp"
#include "mdi/rrt/rrt.hpp"
#include "mdi/rrt/rrt_builder.hpp"
#include "mdi_msgs/RrtFindPath.h"
#include "octomap/octomap_types.h"
#include "ros/init.h"
#include "ros/service_client.h"
#include "ros/service_server.h"

using namespace std::string_literals;

using mdi::Octomap;

// use ptr to forward declare cause otherwise we have to call their constructor,
// which is not allowed before ros::init(...)
std::unique_ptr<ros::ServiceClient> clear_octomap_client, clear_region_of_octomap_client, get_octomap_client;

auto call_get_octomap() -> mdi::Octomap* {
    auto request = octomap_msgs::GetOctomap::Request{};  // empty request
    auto response = octomap_msgs::GetOctomap::Response{};

    get_octomap_client->call(request, response);
    return new mdi::Octomap{response.map};
}

/**
 * @brief reset octomap
 *
 */
auto call_clear_octomap() -> void {
    auto request = std_srvs::Empty::Request{};
    auto response = std_srvs::Empty::Response{};

    clear_octomap_client->call(request, response);
}

/**
 * @brief max and min defines a bbx. All voxels in the bounding box are set to "free"
 */
auto call_clear_region_of_octomap(const octomap::point3d& max, const octomap::point3d& min) -> void {
    const auto convert = [](const octomap::point3d& pt) {
        auto geo_pt = geometry_msgs::Point{};
        geo_pt.x = pt.x();
        geo_pt.y = pt.y();
        geo_pt.z = pt.z();
        return geo_pt;
    };

    auto request = octomap_msgs::BoundingBoxQuery::Request{};
    request.max = convert(max);
    request.min = convert(min);
    auto response = octomap_msgs::BoundingBoxQuery::Response{};  // is empty
    clear_region_of_octomap_client->call(request, response);
}

auto rrt_find_path_handler(mdi_msgs::RrtFindPath::Request& request, mdi_msgs::RrtFindPath::Response& response) -> bool {
    const auto convert = [](const auto& pt) -> mdi::rrt::vec3 {
        return {static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z)};
    };

    const auto start = convert(request.start);
    const auto goal = convert(request.goal);

    auto rrt = mdi::rrt::RRT::from_builder()
                   .start_and_goal_position(start, goal)
                   .max_iterations(request.max_iterations)
                   .goal_bias(request.goal_bias)
                   .probability_of_testing_full_path_from_new_node_to_goal(
                       request.probability_of_testing_full_path_from_new_node_to_goal)
                   .max_dist_goal_tolerance(request.goal_tolerance)
                   .step_size(request.step_size)
                   .build();

    auto success = false;

    auto octomap_ptr = call_get_octomap();

    if (octomap_ptr != nullptr) {
        rrt.assign_octomap(octomap_ptr);

        if (const auto opt = rrt.run()) {
            const auto path = opt.value();
            std::transform(path.begin(), path.end(), std::back_inserter(response.waypoints), [](const auto& pt) {
                auto geo_pt = geometry_msgs::Point{};
                geo_pt.x = pt.x();
                geo_pt.y = pt.y();
                geo_pt.z = pt.z();
                return geo_pt;
            });

            success = true;
        }

        delete octomap_ptr;
    }

    return success;
}

auto main(int argc, char* argv[]) -> int {
    const auto name_of_node = "rrt_service"s;

    ros::init(argc, argv, name_of_node);
    auto nh = ros::NodeHandle();

    get_octomap_client = std::make_unique<ros::ServiceClient>([&] {
        const auto service_name = "/octomap_binary"s;
        return nh.serviceClient<octomap_msgs::GetOctomap>(service_name);
    }());

    clear_octomap_client = std::make_unique<ros::ServiceClient>([&] {
        const auto service_name = "/octomap_server/reset"s;
        return nh.serviceClient<std_srvs::Empty>(service_name);
    }());

    clear_region_of_octomap_client = std::make_unique<ros::ServiceClient>([&] {
        const auto service_name = "/octomap_server/clear_bbx"s;
        return nh.serviceClient<octomap_msgs::BoundingBoxQuery>(service_name);
    }());

    auto service = [&] {
        const auto url = "/rrt_service/find_path"s;
        return nh.advertiseService(url, rrt_find_path_handler);
    }();

    ros::spin();

    return 0;
}
