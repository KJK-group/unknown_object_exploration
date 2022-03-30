#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>

#include "multi_drone_inspection/bezier_spline.hpp"

using mdi::BezierSpline;

ros::Publisher pub_spline;
ros::Publisher pub_spline_input;

auto duration = 5.f;
auto seq = 0;

auto main(int argc, char** argv) -> int {
    ROS_INFO_STREAM("START");
    ros::init(argc, argv, "mdi_spline_tester");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    auto start_time = ros::Time::now();

    pub_spline = nh.advertise<visualization_msgs::Marker>("mdi/visualisation/spline", 10);
    pub_spline_input =
        nh.advertise<visualization_msgs::Marker>("mdi/visualisation/spline_input", 10);

    auto points =
        vector<Vector3f>{Vector3f(0.0, 0.0, 0.0), Vector3f(1.0, 0.5, 0.2), Vector3f(0.5, 1.5, 0.3),
                         Vector3f(0.8, 2.0, 0.7), Vector3f(1.2, 2.2, 1.5), Vector3f(1.0, 3.0, 1.0)};
    auto input_points = vector<geometry_msgs::Point>();
    for (auto& point : points) {
        geometry_msgs::Point p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);
        ROS_INFO_STREAM("input point: " << p);
        input_points.push_back(p);
    }
    visualization_msgs::Marker m;
    m.header.seq = seq++;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.points = input_points;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    pub_spline_input.publish(m);

    std::reverse(points.begin(), points.end());
    ROS_INFO_STREAM("Before constructor");
    auto spline = BezierSpline(points, 40);

    auto spline_points = spline.get_spline_points();
    auto visualisation_points = vector<geometry_msgs::Point>();

    for (int i = 0; i < spline_points.size(); i++) {
        ROS_INFO_STREAM("point:\n");
        ROS_INFO_STREAM("  x: " << spline_points[i](0));
        ROS_INFO_STREAM("  y: " << spline_points[i](1));
        ROS_INFO_STREAM("  z: " << spline_points[i](2));

        geometry_msgs::Point point;
        point.x = spline_points[i](0);
        point.y = spline_points[i](1);
        point.z = spline_points[i](2);
        visualisation_points.push_back(point);

        ros::spinOnce();
        rate.sleep();
    }

    visualization_msgs::Marker marker;
    marker.header.seq = seq++;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.points = visualisation_points;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    pub_spline.publish(marker);

    auto delta_time = ros::Duration();
    // while (ros::ok() && delta_time.toSec() < duration) {
    //     auto delta_time = start_time - ros::Time::now();
    //     auto point = spline.get_point_at_time((float)(delta_time.toSec() / duration));
    //     // ROS_INFO_STREAM("point:\n");
    //     // ROS_INFO_STREAM("  x: " << point(0));
    //     // ROS_INFO_STREAM("  y: " << point(1));
    //     // ROS_INFO_STREAM("  z: " << point(2));

    //     // message
    //     geometry_msgs::PointStamped p;
    //     p.header.seq = seq++;
    //     p.header.frame_id = "map";
    //     p.header.stamp = ros::Time::now();

    //     p.point.x = point(0);
    //     p.point.y = point(1);
    //     p.point.z = point(2);

    //     pub_point.publish(p);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    return 0;
}