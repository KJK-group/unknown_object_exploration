#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

#include "multi_drone_inspection/bezier_spline.hpp"

using mdi::BezierSpline;

ros::Publisher pub_point;

auto duration = 5.f;

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "mdi_spline_tester");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    auto start_time = ros::Time::now();

    pub_point = nh.advertise<geometry_msgs::PointStamped>("mdi/points/spline", 10);

    auto points =
        vector<Vector3f>{Vector3f(0.0, 0.0, 0.0), Vector3f(1.0, 0.5, 0.2), Vector3f(0.5, 1.5, 0.3),
                         Vector3f(0.8, 2.0, 0.7), Vector3f(1.2, 2.2, 1.5), Vector3f(1.0, 3.0, 1.0)};
    auto spline = BezierSpline(points);

    auto delta_time = ros::Duration();
    while (ros::ok() && delta_time.toSec() < duration) {
        auto delta_time = start_time - ros::Time::now();
        spline.get_point_at_time((float)(delta_time.toSec() / duration));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}