cmake_minimum_required(VERSION 3.14)
project(mdi_msgs)

set(MSG_DEPENDENCIES std_msgs geometry_msgs)

find_package(catkin REQUIRED ${MSG_DEPENDENCIES} message_generation)

# find_package(catkin_simple REQUIRED) catkin_simple(ALL_DEPS_REQUIRED)

# message(STATUS "looking for *.srv files")
# file(GLOB SERVICES srv/*.srv)
# message(STATUS ${SERVICES})
# message(STATUS "looking for *.msg files")
# # file(GLOB MESSAGES msg/*.msg)
# set(
#     MESSAGES
#     msg/MissionStateStamped.msg
#     msg/PointNorm.msg
#     msg/PointNormStamped.msg
#     msg/Error.msg
#     msg/ControllerStateStamped.msg
# )
# message(STATUS ${MESSAGES})

add_message_files(
    FILES
    MissionStateStamped.msg
    PointNorm.msg
    PointNormStamped.msg
    Error.msg
    ControllerStateStamped.msg)
add_service_files(FILES RrtFindPath.srv)

generate_messages(DEPENDENCIES ${MSG_DEPENDENCIES})

catkin_package(CATKIN_DEPENDS message_runtime)