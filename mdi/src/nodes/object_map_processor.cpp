#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_server/OctomapServer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "mdi/octomap.hpp"
#include "mdi/utils/utils.hpp"
#include "mdi_msgs/ControllerStateStamped.h"
#include "mdi_msgs/Model.h"

// zed-ros-wrapper
// http://wiki.ros.org/zed-ros-wrapper
// CITE

// point cloud library
// https://pointclouds.org/documentation/index.html
// CITE

mdi::Octomap* object_map;
auto get_object_map(octomap_msgs::GetOctomap::Request& request, octomap_msgs::GetOctomap::Response& response) -> bool {
    return octomap_msgs::binaryMapToMsg(object_map->octree(), response.map);
};

pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_point_cloud{};
/**
 * @brief callback for /zed/rgb/image/rect/color converts sensor_msgs::PointCloud2 to pcl::PointCloud2
 *
 * @param cloud message from /zed/rgb/image/rect/color
 * @return void
 */
auto point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud) -> void {
    pcl::PCLPointCloud2::Ptr pc2{};
    pcl_conversions::toPCL(*cloud, *pc2);
    pcl::fromPCLPointCloud2(*pc2, *unfiltered_point_cloud);
}
sensor_msgs::Image raw_image;
auto image_cb(const sensor_msgs::Image::ConstPtr& image) -> void { raw_image = *image; }

auto main(int argc, char* argv[]) -> int {
    // ros
    ros::init(argc, argv, "mdi_object_map_service");
    auto nh = ros::NodeHandle();
    ros::Rate rate(mdi::utils::DEFAULT_LOOP_RATE);

    auto sub_camera_rgb =
        nh.subscribe<sensor_msgs::Image>("/zed/rgb/image_rect_color", mdi::utils::DEFAULT_QUEUE_SIZE, image_cb);
    auto sub_camera_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
        "/zed/point_cloud/cloud_registered ", mdi::utils::DEFAULT_QUEUE_SIZE, point_cloud_cb);
    auto client_model = nh.serviceClient<mdi_msgs::Model>("/mdi/model");

    double resolution = 0;
    double thresh_prob_min = 0.1;
    double thresh_prob_max = 0.5;
    // arguments
    if (argc > 1) resolution = std::stod(argv[1]);
    if (argc > 2) thresh_prob_min = std::stod(argv[2]);
    if (argc > 3) thresh_prob_max = std::stod(argv[3]);

    object_map = new mdi::Octomap{resolution, thresh_prob_min, thresh_prob_max};

    nh.advertiseService<octomap_msgs::GetOctomap::Request, octomap_msgs::GetOctomap::Response>("/mdi/object_map",
                                                                                               get_object_map);

    while (ros::ok()) {
        // point cloud filter with model output
        auto current_image = raw_image;
        auto id = current_image.header.seq;
        auto current_point_cloud = unfiltered_point_cloud;

        mdi_msgs::Model srv;
        srv.request.image = current_image;

        // using pcl library to filter point cloud
        if (client_model.call(srv)) {
            // fill the filter with the segmented indices from the model output
            auto indices_filter = pcl::PointIndices{};
            for (int i = 0; i < srv.response.data.size(); i++) {
                indices_filter.indices.push_back(i);
            }

            // construct index filter
            auto filter = pcl::ExtractIndices<pcl::PointXYZ>();
            filter.setInputCloud(unfiltered_point_cloud);
            filter.setIndices(boost::make_shared<const pcl::PointIndices>(indices_filter));
            // perform filtration and output to filtered cloud
            auto filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr{};
            filter.filter(*filtered_cloud);

            // inserting the filtered point cloud
            object_map->insert_points(filtered_cloud->points);
        }
    }

    return 0;
}