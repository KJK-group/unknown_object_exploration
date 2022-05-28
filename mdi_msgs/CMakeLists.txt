cmake_minimum_required(VERSION 3.14)
project(mdi_msgs)

set(MSG_DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

find_package(catkin REQUIRED ${MSG_DEPENDENCIES} message_generation)

add_message_files(
  FILES
  MissionStateStamped.msg
  PointNorm.msg
  PointNormStamped.msg
  Error.msg
  ControllerStateStamped.msg
  Degree.msg
  DepthRange.msg
  FoV.msg
  NbvConfig.msg
  RrtConfig.msg
  Waypoints.msg
)

add_service_files(
    FILES
    Model.srv
    RrtFindPath.srv
    NBV.srv
)

generate_messages(DEPENDENCIES ${MSG_DEPENDENCIES})

catkin_package(CATKIN_DEPENDS message_runtime)
