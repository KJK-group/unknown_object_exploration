#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

#include <algorithm>

#include "multi_drone_inspection/bezier_spline.hpp"

using mdi::BezierSpline;

ros::Publisher pub_point;

auto duration = 5.f;
auto seq = 0;

auto main(int argc, char** argv) -> int {
    ROS_INFO_STREAM("START");
    ros::init(argc, argv, "mdi_spline_tester");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    auto start_time = ros::Time::now();

    pub_point = nh.advertise<geometry_msgs::PointStamped>("mdi/points/spline", 10);

    auto points =
        vector<Vector3f>{Vector3f(0.0, 0.0, 0.0), Vector3f(1.0, 0.5, 0.2), Vector3f(0.5, 1.5, 0.3),
                         Vector3f(0.8, 2.0, 0.7), Vector3f(1.2, 2.2, 1.5), Vector3f(1.0, 3.0, 1.0)};
    std::reverse(points.begin(), points.end());
    ROS_INFO_STREAM("Before constructor");
    auto spline = BezierSpline(points);

    auto spline_points = spline.get_spline_points();
    for (int i = 0; i < spline_points.size(); i++) {
        ROS_INFO_STREAM("point:\n");
        ROS_INFO_STREAM("  x: " << spline_points[i](0));
        ROS_INFO_STREAM("  y: " << spline_points[i](1));
        ROS_INFO_STREAM("  z: " << spline_points[i](2));

        geometry_msgs::PointStamped p;
        p.header.seq = seq++;
        p.header.frame_id = "map";
        p.header.stamp = ros::Time::now();

        p.point.x = spline_points[i](0);
        p.point.y = spline_points[i](1);
        p.point.z = spline_points[i](2);
        pub_point.publish(p);
        ros::spinOnce();
        rate.sleep();
    }
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