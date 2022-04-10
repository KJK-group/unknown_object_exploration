#ifndef _MULTI_DRONE_INSPECTION_TF2_HPP_
#define _MULTI_DRONE_INSPECTION_TF2_HPP_

namespace mdi::utils::tf2 {

// loop_rate.sleep();
auto lookup_transform(const tf2::TransformListener& listener) -> tf2::TransformListener {}

// try {
//   listener.waitForTransform("camera", "base_link", ros::Time::now(),
//                             ros::Duration(3.0));
//   listener.lookupTransform("camera", "base_link", ros::Time(0),
//                            stamped_transform);
// } catch (tf::TransformException &ex) {
//   ROS_WARN("%s", ex.what());
// }
}  // namespace mdi::utils::tf2

#endif  // _MULTI_DRONE_INSPECTION_TF2_HPP_
