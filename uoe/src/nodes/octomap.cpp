

#include <ros/ros.h>

auto main(int argc, char* argv[]) -> int {
	ros::init(argc, argv, "octomap_node");
	auto nh = ros::NodeHandle();


	ros::spin();
	return 0;
}
