cmake_minimum_required(VERSION 3.14)
project(uoe_msgs)

set(MSG_DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

find_package(catkin REQUIRED ${MSG_DEPENDENCIES} message_generation)

add_message_files(
  FILES
  ControllerStateStamped.msg
  Degree.msg
  DepthRange.msg
  DroneConfig.msg
  Error.msg
  FoV.msg
  FoVGainMetric.msg
  MissionStateStamped.msg
  NbvConfig.msg
  PointNorm.msg
  PointNormStamped.msg
  FovVolume.msg
  VoxelVolume.msg
  ObjectMapCompleteness.msg
  GainScore.msg
  RrtConfig.msg
  Waypoints.msg)

add_service_files(FILES Model.srv NBV.srv RrtFindPath.srv)

generate_messages(DEPENDENCIES ${MSG_DEPENDENCIES})

catkin_package(CATKIN_DEPENDS message_runtime)
