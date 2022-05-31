#include <octomap/OcTree.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>

// #include "Eigen/src/Geometry/AngleAxis.h"
#include "mdi/bbx.hpp"
#include "mdi/common_headers.hpp"
#include "mdi/common_types.hpp"
#include "mdi/gain.hpp"
#include "mdi/octomap.hpp"
#include "mdi/rrt/rrt.hpp"
#include "mdi/rrt/rrt_builder.hpp"
#include "mdi/utils/rviz.hpp"
#include "mdi/utils/transform.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/FoVGainMetric.h"
#include "mdi_msgs/NBV.h"
#include "mdi_msgs/RrtFindPath.h"
#include "octomap/octomap_types.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_client.h"
#include "ros/service_server.h"

#ifdef VISUALIZE_MARKERS_IN_RVIZ
#include "mdi/visualization/bbx.hpp"
#include "mdi/visualization/fov.hpp"

std::unique_ptr<ros::Publisher> nbv_metric_pub;
std::unique_ptr<ros::Publisher> marker_pub;
std::unique_ptr<ros::Publisher> marker_array_pub;

// marker_array.markers
// visualization_msgs::MarkerArray marker_array;

#endif  // VISUALIZE_MARKERS_IN_RVIZ

using namespace std::string_literals;
using vec3 = Eigen::Vector3f;

using mdi::Octomap;
using namespace mdi::types;

// use ptr to forward declare cause otherwise we have to call their constructor,
// which is not allowed before ros::init(...)
std::unique_ptr<ros::ServiceClient> clear_octomap_client, clear_region_of_octomap_client,
    get_octomap_client;
std::unique_ptr<ros::Publisher> waypoints_path_pub;
using waypoint_cb = std::function<void(const vec3&, const vec3&)>;
using new_node_cb = std::function<void(const vec3&, const vec3&)>;
using raycast_cb = std::function<void(const vec3&, const vec3&, const float, bool)>;
waypoint_cb before_waypoint_optimization;
waypoint_cb after_waypoint_optimization;
new_node_cb new_node_created;
raycast_cb raycast;

auto call_get_environment_octomap() -> mdi::Octomap* {
    auto request = octomap_msgs::GetOctomap::Request{};  // empty request
    auto response = octomap_msgs::GetOctomap::Response{};

    get_octomap_client->call(request, response);
    return response.map.data.size() == 0 ? nullptr : new mdi::Octomap{response.map};
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
auto call_clear_region_of_octomap(const octomap::point3d& max, const octomap::point3d& min)
    -> void {
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

auto waypoints_to_geometry_msgs_points(const mdi::rrt::RRT::Waypoints& wps)
    -> std::vector<geometry_msgs::Point> {
    auto points = std::vector<geometry_msgs::Point>();
    std::transform(wps.begin(), wps.end(), std::back_inserter(points), [](const auto& pt) {
        auto geo_pt = geometry_msgs::Point{};
        geo_pt.x = pt.x();
        geo_pt.y = pt.y();
        geo_pt.z = pt.z();
        return geo_pt;
    });

    return points;
}

auto rrt_find_path_handler(mdi_msgs::RrtFindPath::Request& request,
                           mdi_msgs::RrtFindPath::Response& response) -> bool {
    ROS_INFO("rrt service called.");

    const auto geometry_msgs_point_to_vec3 = [](const auto& pt) -> mdi::rrt::vec3 {
        return {static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z)};
    };

    const auto start = geometry_msgs_point_to_vec3(request.rrt_config.start);
    const auto goal = geometry_msgs_point_to_vec3(request.rrt_config.goal);

    auto rrt = mdi::rrt::RRT::from_builder()
                   .start_and_goal_position(start, goal)
                   .max_iterations(request.rrt_config.max_iterations)
                   .goal_bias(request.rrt_config.goal_bias)
                   .probability_of_testing_full_path_from_new_node_to_goal(
                       request.rrt_config.probability_of_testing_full_path_from_new_node_to_goal)
                   .max_dist_goal_tolerance(request.rrt_config.goal_tolerance)
                   .step_size(request.rrt_config.step_size)
                   .drone_width(request.drone_config.width)
                   .drone_height(request.drone_config.height)
                   .drone_depth(request.drone_config.depth)
                   .build();

    // rrt.register_cb_for_event_on_new_node_created(
    //     [](const auto& p1, const auto& p2) { std::cout << "New node created" << '\n'; });

    // rrt.register_cb_for_event_on_new_node_created(new_node);

    auto octomap_ptr = call_get_environment_octomap();

    if (octomap_ptr != nullptr) {
        ROS_INFO("there is a octomap available");
        rrt.assign_octomap(octomap_ptr);
    }
    auto found_a_path = false;

    ROS_INFO("running rrt");
    if (const auto opt = rrt.run()) {
        ROS_INFO("rrt: found a path");

        const auto path = opt.value();
        std::cout << "path.size() = " << path.size() << std::endl;
        response.waypoints = waypoints_to_geometry_msgs_points(path);

        found_a_path = true;
    }

    if (octomap_ptr != nullptr) {
        ROS_INFO("deallocating octomap copy");
        delete octomap_ptr;
    }

    return found_a_path;
}

auto nbv_handler(mdi_msgs::NBV::Request& request, mdi_msgs::NBV::Response& response) -> bool {
    ROS_INFO("nbv request received");

    const auto geometry_msgs_point_to_vec3 = [](const auto& pt) -> mdi::rrt::vec3 {
        return {static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z)};
    };

    // these parameters are static so we only compute them once
    const auto horizontal = FoVAngle::from_degrees(request.fov.horizontal.angle);
    const auto vertical = FoVAngle::from_degrees(request.fov.vertical.angle);
    const auto depth_range = DepthRange{request.fov.depth_range.min, request.fov.depth_range.max};

    auto rrt = mdi::rrt::RRT::from_builder()
                   .start_and_goal_position(
                       geometry_msgs_point_to_vec3(request.rrt_config.start),
                       geometry_msgs_point_to_vec3(
                           request.rrt_config.goal))  // goal is irrelevant - not really though
                   .max_iterations(request.rrt_config.max_iterations)
                   .goal_bias(request.rrt_config.goal_bias)
                   .probability_of_testing_full_path_from_new_node_to_goal(0.0)
                   .max_dist_goal_tolerance(/ 0.0)
                   .step_size(request.rrt_config.step_size)
                   .drone_width(request.drone_config.width)
                   .drone_height(request.drone_config.height)
                   .drone_depth(request.drone_config.depth)
                   .build();

    ROS_INFO_STREAM("" << rrt);

    auto octomap_environment_ptr = call_get_environment_octomap();

    if (octomap_environment_ptr != nullptr) {
        ROS_INFO("there is a octomap available");
        rrt.assign_octomap(octomap_environment_ptr);
        ROS_INFO_STREAM(
            "size of octomap (in bytes) is: " << octomap_environment_ptr->octree().memoryUsage());
    }

    // TODO: how to initialize this maybe std::optional ?
    auto best_fov_gain_metric = mdi::FoVGainMetric{};
    double best_gain = std::numeric_limits<double>::lowest();
    vec3 best_point = geometry_msgs_point_to_vec3(request.rrt_config.start);
    const vec3 target = geometry_msgs_point_to_vec3(request.rrt_config.goal);
    // vec3{request.rrt_config.start.x, request.rrt_config.start.y, request.rrt_config.start.z};

    bool found_suitable_nbv = false;

#ifdef VISUALIZE_MARKERS_IN_RVIZ
    // visualization_msgs::MarkerArray marker_array;
    // auto new_node_arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
    //                                   .arrow_head_width(0.05f)
    //                                   .arrow_length(0.1f)
    //                                   .arrow_width(0.1f)
    //                                   .color({0, 1, 0, 1})
    //                                   .build();
    // rrt.register_cb_for_event_on_new_node_created([&](const auto& from, const auto& to) {
    //     auto msg = new_node_arrow_msg_gen({from, to});
    //     marker_array.markers.push_back(msg);
    // });

    // // TODO: comment
    // rrt.register_cb_for_event_before_optimizing_waypoints(before_waypoint_optimization);
    // rrt.register_cb_for_event_after_optimizing_waypoints(after_waypoint_optimization);
    // rrt.register_cb_for_event_on_raycast(raycast);
#endif  // VISUALIZE_MARKERS_IN_RVIZ

    rrt.register_cb_for_event_on_new_node_created([&](const auto&, const vec3& new_point) {
        vec3 dir = target - new_point;

        const auto yaw = std::atan2(dir.y(), dir.x());
        // construct fov
        Eigen::Quaternionf orientation =
            Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(deg2rad(-request.fov.pitch.angle), Eigen::Vector3f::UnitY());

        const auto pose = Pose{new_point, orientation};
        const auto fov = FoV{pose, horizontal, vertical, depth_range, target};

#ifdef VISUALIZE_MARKERS_IN_RVIZ

        // mdi::visualization::visualize_fov(fov, *marker_pub);
        // mdi::visualization::visualize_bbx(mdi::compute_bbx(fov), *marker_pub);
        /* mdi::visualization::visualize_voxels_inside_fov(
            fov, *octomap_environment_ptr, octomap_environment_ptr->resolution(),
            *marker_array_pub); */
#endif  // VISUALIZE_MARKERS_IN_RVIZ

        // ROS_INFO("calculating gain of fov");
        const auto fov_gain_metric = mdi::gain_of_fov(
            fov, *octomap_environment_ptr, request.nbv_config.weight_free,
            request.nbv_config.weight_occupied, request.nbv_config.weight_unknown,
            request.nbv_config.weight_distance_to_object, request.nbv_config.weight_not_visible,
            mdi::utils::transform::geometry_mgs_point_to_vec(request.rrt_config.start),
            [](double x) { return x /* x * x */; },
            [](const mdi::types::vec3&, const mdi::types::vec3&, float, const bool) {});

        const double gain = fov_gain_metric.gain_total;

        if (gain > best_gain) {
            ROS_INFO_STREAM("gain (" << std::to_string(gain)
                                     << ") is better that the current best gain ("
                                     << std::to_string(best_gain) << ")");
            best_fov_gain_metric = fov_gain_metric;
            best_gain = gain;
            best_point = fov.pose().position;
        }

        if (gain >= request.nbv_config.gain_of_interest_threshold) {
            ROS_INFO_STREAM("found a suitable gain " << std::to_string(gain));
            found_suitable_nbv = true;
        }
    });

    // grow rrt incrementally
    for (std::size_t i = 0; i < request.rrt_config.max_iterations; ++i) {
        rrt.grow1();

        if (found_suitable_nbv) {
            ROS_INFO("found suitable nbv");
            // backtrack and set waypoints
            if (const auto opt = rrt.waypoints_from_newest_node()) {
                const auto path = opt.value();
                response.waypoints = waypoints_to_geometry_msgs_points(path);
                response.found_nbv_with_sufficent_gain = true;
            } else {
                // should not happen, but just in case
                response.found_nbv_with_sufficent_gain = false;
            }

            break;
        }
    }

    if (! response.found_nbv_with_sufficent_gain) {
        // a suitable nbv has not been found, so we use best found instead
        ROS_INFO("no suitable nbv found, use best found instead");

        if (const auto opt = rrt.get_waypoints_from_nearsest_node_to(best_point)) {
            const auto path = opt.value();
            response.waypoints = waypoints_to_geometry_msgs_points(path);
        }
    }

    nbv_metric_pub->publish(mdi::to_ros_msg(best_fov_gain_metric));
    ros::spinOnce();
    ros::Rate(10).sleep();

#ifdef VISUALIZE_MARKERS_IN_RVIZ
    // std::cout << "publishing RRT tree..." << std::endl;
    // marker_array_pub->publish(marker_array);
    // ros::spinOnce();
    // ros::Rate(mdi::utils::DEFAULT_LOOP_RATE).sleep();
    // marker_array.markers.clear();

    vec3 dir = target - mdi::utils::transform::geometry_mgs_point_to_vec(response.waypoints.back());

    const auto yaw = std::atan2(dir.y(), dir.x());
    // construct fov
    Eigen::Quaternionf orientation =
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(deg2rad(-request.fov.pitch.angle), Eigen::Vector3f::UnitY());

    const auto pose = Pose{
        mdi::utils::transform::geometry_mgs_point_to_vec(response.waypoints.back()), orientation};
    const auto fov = FoV{pose, horizontal, vertical, depth_range, target};

    auto ray_marker_array = visualization_msgs::MarkerArray{};
    auto raycast_arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                                     .arrow_head_width(0.15f)
                                     .arrow_length(0.3f)
                                     .arrow_width(0.05f)
                                     .color({0, 1, 0, 1})
                                     .build();

    marker_array_pub->publish(ray_marker_array);
    ros::spinOnce();
    ros::Rate(10).sleep();
#endif  // VISUALIZE_MARKERS_IN_RVIZ

    std::cout << yaml(best_fov_gain) << std::endl;

    if (octomap_environment_ptr != nullptr) {
        ROS_INFO("deallocating octomap copy");
        delete octomap_environment_ptr;
    }

    ROS_INFO_STREAM("information gain: " << best_gain);

    return true;
}

auto main(int argc, char* argv[]) -> int {
    const auto name_of_node = "rrt_service"s;

    ros::init(argc, argv, name_of_node);
    auto nh = ros::NodeHandle();

#ifdef VISUALIZE_MARKERS_IN_RVIZ
    // marker_array.markers = std::vector<visualization_msgs::Marker>{};
    auto rate = ros::Rate(mdi::utils::DEFAULT_LOOP_RATE);

    waypoints_path_pub = std::make_unique<ros::Publisher>([&] {
        const auto service_name = "/visualization_marker"s;
        return nh.advertise<visualization_msgs::Marker>(service_name, 10);
    }());

    auto before_waypoint_optimization_arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                                                          .arrow_head_width(0.05f)
                                                          .arrow_length(0.01f)
                                                          .arrow_width(0.05f)
                                                          .color({0, 1, 0, 1})
                                                          .build();

    before_waypoint_optimization = [&](const vec3& from, const vec3& to) {
        auto msg = before_waypoint_optimization_arrow_msg_gen({from, to});
        waypoints_path_pub->publish(msg);
        rate.sleep();
        ros::spinOnce();
    };

    ros::Duration(1).sleep();

    auto after_waypoint_optimization_arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                                                         .arrow_head_width(0.1f)
                                                         .arrow_length(0.01f)
                                                         .arrow_width(0.1f)
                                                         .color({0, 0, 1, 1})
                                                         .build();

    after_waypoint_optimization = [&](const vec3& from, const vec3& to) {
        auto msg = after_waypoint_optimization_arrow_msg_gen({from, to});
        waypoints_path_pub->publish(msg);
        rate.sleep();
        ros::spinOnce();
    };

    ros::Duration(1).sleep();
    auto raycast_arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                                     .arrow_head_width(0.15f)
                                     .arrow_length(0.3f)
                                     .arrow_width(0.05f)
                                     .color({0, 1, 0, 1})
                                     .build();

    raycast = [&](const vec3& origin, const vec3& direction, const float length, bool did_hit) {
        auto msg = raycast_arrow_msg_gen({origin, origin + direction.normalized() * length});
        if (did_hit) {
            return;
            msg.color.r = 1;
            msg.color.g = 0;
        }
        std::cout << "PUBLISHIN RAYCAST" << std::endl;
        waypoints_path_pub->publish(msg);
        rate.sleep();
        ros::spinOnce();
    };

    auto new_node_arrow_msg_gen = mdi::utils::rviz::arrow_msg_gen::builder()
                                      .arrow_head_width(0.05f)
                                      .arrow_length(0.1f)
                                      .arrow_width(0.1f)
                                      .color({0, 1, 0, 1})
                                      .build();

    new_node_created = [&](const vec3& from, const vec3& to) {
        auto msg = new_node_arrow_msg_gen({from, to});
        // marker_array.markers.push_back(msg);
        waypoints_path_pub->publish(msg);
        rate.sleep();
        ros::spinOnce();
    };

    marker_pub = std::make_unique<ros::Publisher>([&] {
        const auto topic_name = "/visualization_marker";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::Marker>(topic_name, queue_size);
    }());

    marker_array_pub = std::make_unique<ros::Publisher>([&] {
        const auto topic_name = "/visualization_marker_array";
        const auto queue_size = 10;
        return nh.advertise<visualization_msgs::MarkerArray>(topic_name, queue_size);
    }());

#endif  // VISUALIZE_MARKERS_IN_RVIZ

    get_octomap_client = std::make_unique<ros::ServiceClient>([&] {
        const auto service_name = "/environment_voxel_map/octomap_binary"s;
        return nh.serviceClient<octomap_msgs::GetOctomap>(service_name);
    }());

    clear_octomap_client = std::make_unique<ros::ServiceClient>([&] {
        const auto service_name = "/environment_voxel_map/environment_voxel_map/reset"s;
        return nh.serviceClient<std_srvs::Empty>(service_name);
    }());

    clear_region_of_octomap_client = std::make_unique<ros::ServiceClient>([&] {
        const auto service_name = "/environment_voxel_map/environment_voxel_map/clear_bbx"s;
        return nh.serviceClient<octomap_msgs::BoundingBoxQuery>(service_name);
    }());

    ROS_INFO("creating rrt service");

    auto service = [&] {
        const auto url = "/mdi/rrt_service/find_path";
        return nh.advertiseService(url, rrt_find_path_handler);
    }();

    ROS_INFO("creating nbv service");

    auto nbv_service = [&] {
        const auto url = "/mdi/rrt_service/nbv";
        return nh.advertiseService(url, nbv_handler);
    }();

    nbv_metric_pub = std::make_unique<ros::Publisher>([&] {
        const auto topic_name = "/mdi/best_nbv_fov_metric"s;
        const auto queue_size = 10;
        return nh.advertise<mdi_msgs::FoVGainMetric>(topic_name, queue_size);
    }());

    ros::spin();

    return 0;
}
