#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>

#include "multi_drone_inspection/bezier_spline.hpp"

using mdi::BezierSpline;
using std::abs;

ros::Publisher pub_line;
ros::Publisher pub_spline_anim;

auto duration = 5.f;
auto input_seq = 0;
auto output_seq = 0;
auto anim_seq = 0;
auto anim_seq2 = 0;

auto main(int argc, char** argv) -> int {
    ROS_INFO_STREAM("START");
    ros::init(argc, argv, "mdi_spline_tester");
    auto nh = ros::NodeHandle();
    ros::Rate rate(20.0);
    auto start_time = ros::Time::now();

    pub_line = nh.advertise<visualization_msgs::Marker>("mdi/visualisation/spline", 10);
    pub_spline_anim = nh.advertise<visualization_msgs::Marker>("mdi/visualisation/spline_anim", 10);

    auto points = vector<Vector3f>{Vector3f(0.0, 0.0, 0.0),  Vector3f(3.0, 0.5, 1.0),
                                   Vector3f(-3.5, 1.5, 0.0), Vector3f(-2.8, 1.0, 0.7),
                                   Vector3f(1.2, 2.2, 1.5),  Vector3f(1.0, 3.0, 1.0)};

    ROS_INFO_STREAM("INPUT POINTS");
    auto input_points = vector<geometry_msgs::Point>();
    for (auto& point : points) {
        point *= 10;
        geometry_msgs::Point p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);
        input_points.push_back(p);

        ros::spinOnce();
        rate.sleep();
    }
    for (int i = 0; i < input_points.size(); i++) {
        ROS_INFO_STREAM("input_point[" << i << "] \n" << input_points[i]);
    }

    visualization_msgs::Marker input_line_marker;
    input_line_marker.header.seq = input_seq++;
    input_line_marker.header.frame_id = "map";
    input_line_marker.header.stamp = ros::Time();
    input_line_marker.id = 0;
    input_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    input_line_marker.color.a = 0.5;
    input_line_marker.color.r = 1.0;
    input_line_marker.color.g = 1.0;
    input_line_marker.color.b = 1.0;
    input_line_marker.scale.x = 0.1;
    input_line_marker.points = input_points;
    input_line_marker.action = 0;

    std::reverse(points.begin(), points.end());
    ROS_INFO_STREAM("BEFORE SPLINE CONSTRUCTOR");
    auto spline = BezierSpline(points, 40);

    auto spline_points = spline.get_spline_points();
    auto visualisation_points = vector<geometry_msgs::Point>();

    for (int i = 0; i < spline_points.size(); i++) {
        // ROS_INFO_STREAM("point:\n");
        // ROS_INFO_STREAM("  x: " << spline_points[i](0));
        // ROS_INFO_STREAM("  y: " << spline_points[i](1));
        // ROS_INFO_STREAM("  z: " << spline_points[i](2));

        geometry_msgs::Point point;
        point.x = spline_points[i](0);
        point.y = spline_points[i](1);
        point.z = spline_points[i](2);
        visualisation_points.push_back(point);

        ros::spinOnce();
        rate.sleep();
    }

    visualization_msgs::Marker marker;
    marker.header.seq = output_seq++;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.points = visualisation_points;
    marker.scale.x = 0.1;
    // marker.lifetime = ros::Duration(10);
    input_line_marker.action = 0;
    pub_line.publish(marker);
    pub_line.publish(input_line_marker);

    ROS_INFO_STREAM("AFTER SPLINE VISUALISATION");

    visualization_msgs::Marker anim_marker;
    anim_marker.header.seq = anim_seq++;
    anim_marker.header.frame_id = "map";
    anim_marker.header.stamp = ros::Time();
    marker.id = 2;
    anim_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    anim_marker.color.a = 1.0;
    anim_marker.color.r = 0.0;
    anim_marker.color.g = 1.0;
    anim_marker.color.b = 1.0;
    anim_marker.scale.x = 0.5;
    anim_marker.scale.y = 0.5;
    anim_marker.scale.z = 0.5;

    ROS_INFO_STREAM("arc length: " << spline.get_length());

    auto forwards = true;
    auto spline_length = spline.get_length();

    auto duration = ros::Duration(5.0);
    auto speed = spline_length / duration.toSec();  // 5 m/s
    auto st = ros::Time::now();
    ros::Duration dt;
    while (ros::ok()) {
        if (dt > duration) {
            st = ros::Time::now();
            forwards = !forwards;
        }
        dt = ros::Time::now() - st;

        auto point1 = spline.f(dt.toSec() / duration.toSec());
        geometry_msgs::Point p1;
        p1.x = point1(0);
        p1.y = point1(1);
        p1.z = point1(2);

        auto distance = speed * dt.toSec();
        if (forwards) {
            distance = spline_length - distance;
            if (distance < 0) {
                distance = 0;
            }
        }
        // ROS_INFO_STREAM("distance: " << distance);
        auto point2 = spline.get_point_at_distance(distance);
        geometry_msgs::Point p2;
        p2.x = point2(0);
        p2.y = point2(1);
        p2.z = point2(2);

        anim_marker.points = vector<geometry_msgs::Point>{p2};
        anim_marker.header.seq = anim_seq++;
        pub_spline_anim.publish(anim_marker);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}