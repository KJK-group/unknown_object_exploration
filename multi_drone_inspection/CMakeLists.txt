cmake_minimum_required(VERSION 3.14)
project(
  multi_drone_inspection
  VERSION 0.1
  DESCRIPTION ""
  LANGUAGES CXX)

add_compile_options(-std=c++17)
# global configuration set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")
# set(CMAKE_CXX_STANDARD 17) set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_EXPORT_COMPILE_COMMANDS 1) set(CMAKE_EXPORT_COMPILE_COMMANDS ON
# CACHE INTERNAL "")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             nav_msgs
             mavros_msgs
             mdi_msgs
             roscpp
             tf2
             tf2_ros
             tf2_eigen)
# voxblox_ros)
find_package(Eigen3 REQUIRED)
find_package(xacro REQUIRED)

add_subdirectory(deps/fmt)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# dependencies
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(libs/kdtree)

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# add_message_files( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg FILES
# MissionStateStamped.msg PointNormStamped.msg ) generate_messages(DEPENDENCIES
# geometry_msgs std_msgs)

# ##############################################################################
# Declare ROS dynamic reconfigure parameters ##
# ##############################################################################

catkin_package()

# ##############################################################################
# Build ##
# ##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

function(add_ros_executable name main)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${main})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES} fmt::fmt-header-only)
  target_compile_options(
    ${name}
    PRIVATE -O0
            # -g -Wall -Werror -Wextra -Wpedantic -Weffc++ -Wsign-conversion
  )
endfunction(add_ros_executable)

# add_ros_executable(visualize_kdtree src/visualize_kdtree.cpp)
# add_ros_executable(linear_search_vs_kdtree test/linear_search_vs_kdtree.cpp)
# add_ros_executable(test_rrt test/rrt.test.cpp)

add_executable(test_rrt test/rrt.test.cpp)
add_dependencies(test_rrt ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_sources(test_rrt PUBLIC src/rrt/rrt.cpp)
target_link_libraries(test_rrt ${catkin_LIBRARIES} fmt::fmt-header-only
                      kdtree-header-only)

# target_compile_options(test_rrt PRIVATE -g -O0 -Wall -Wextra -Wpedantic)

target_compile_options(test_rrt PRIVATE -O3 -finline-functions-called-once)
# target_compile_options(test_rrt PRIVATE -g -O0 -Wall -Wextra -Wpedantic)
# target_compile_definitions(test_rrt PUBLIC MEASURE_PERF)
target_compile_definitions(test_rrt PUBLIC USE_KDTREE)
# target_sources(visualize_kdtree PUBLIC src/kdtree3/kdtree3.cpp)
# target_compile_options(visualize_kdtree PRIVATE -g)

add_ros_executable(test_controller src/test_controller.cpp)
add_ros_executable(trajectory_follower src/trajectory_follower.cpp)
add_ros_executable(spline_tester src/spline_tester.cpp)
add_ros_executable(mission_state src/mission_state.cpp)
add_ros_executable(pid_controller src/pid_controller.cpp)
add_ros_executable(publish_traversed_path src/publish_traversed_path.cpp)

target_sources(
  spline_tester
  PUBLIC src/bezier_spline.cpp
  PUBLIC src/math.cpp)

target_sources(
  trajectory_follower
  PUBLIC src/bezier_spline.cpp
  PUBLIC src/math.cpp)

target_sources(
  mission_state
  PUBLIC src/bezier_spline.cpp
  PUBLIC src/transformlistener.cpp
  PUBLIC src/math.cpp
  PUBLIC src/rrt/rrt.cpp)

target_sources(pid_controller PUBLIC src/transformlistener.cpp)

# ##############################################################################
# Install ##
# ##############################################################################

install(PROGRAMS scripts/make_px4_airsim.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/make_px4_gazebo.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
