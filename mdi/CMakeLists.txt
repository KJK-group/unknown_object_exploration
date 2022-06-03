cmake_minimum_required(VERSION 3.14)
project(
  mdi
  VERSION 0.1
  DESCRIPTION ""
  LANGUAGES CXX)

# ##############################################################################
# GLOBAL CONFIGURATION
# ##############################################################################
# set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ##############################################################################
# SYSTEM DEPENDENCIES
# ##############################################################################

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             nav_msgs
             mavros_msgs
             octomap_msgs
             sensor_msgs
             octomap_ros
             mdi_msgs
             roscpp
             tf2
             tf2_ros
             tf2_eigen)

include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

# find_package(PCL 1.0 REQUIRED COMPONENTS io)
# include_directories(${PCL_INCLUDE_DIRS}) link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS}) find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS}) link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# file(GLOB PCL_LIBRARIES /usr/local/lib/libpcl_*) set(PCL_INCLUDE_DIRS
# /usr/local/include/pcl-1.12)

# include_directories(${PROJECT_NAME} ${PCL_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

# ##############################################################################
# LOCAL DEPENDENCIES
# ##############################################################################

include_directories(include)

add_subdirectory(libs/kdtree)

catkin_package()

# ##############################################################################
# BUILD
# ##############################################################################

function(enable_optimization target)
  target_compile_options(${target} PRIVATE -O2 -finline-functions-called-once)
endfunction(enable_optimization)

function(enable_pedantic_warnings target)
  target_compile_options(
    ${target} PRIVATE -Wall -Wextra
                      # -Wpedantic # -Weffc++ -Wsign-conversion
  )
endfunction(enable_pedantic_warnings)

function(add_ros_dependencies target)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
endfunction(add_ros_dependencies)

function(add_ros_executable target main)
  add_executable(${target} ${CMAKE_CURRENT_SOURCE_DIR}/${main})
  add_ros_dependencies(${target})
  enable_optimization(${target})
  enable_pedantic_warnings(${target})
  target_link_libraries(${target} ${catkin_LIBRARIES})
endfunction(add_ros_executable)

# ##############################################################################
# LIBRARIES
# ##############################################################################

# bezier library
add_library(bezier_spline SHARED src/bezier_spline.cpp)
enable_optimization(bezier_spline)

# linear trajectory library
add_library(linear_trajectory SHARED src/linear_trajectory.cpp)
enable_optimization(linear_trajectory)

# rrt library
add_library(rrt SHARED src/rrt.cpp)
target_link_libraries(rrt ${catkin_LIBRARIES} kdtree-header-only)
enable_optimization(rrt)

# compound trajectory library
add_library(compound_trajectory SHARED src/compound_trajectory.cpp)
target_link_libraries(compound_trajectory bezier_spline linear_trajectory rrt)
enable_optimization(compound_trajectory)

# mission library
add_library(mission SHARED src/mission.cpp)
target_link_libraries(mission compound_trajectory rrt)
enable_optimization(mission)

# control library
add_library(control SHARED src/pid_controller.cpp)
target_sources(control PUBLIC src/transformlistener.cpp)
enable_optimization(control)

# ##############################################################################
# EXECUTABLE ROS NODES
# ##############################################################################

add_ros_executable(control_manager src/nodes/control_manager.cpp)
target_link_libraries(control_manager control)

add_ros_executable(mission_manager src/nodes/mission_manager.cpp)
target_link_libraries(mission_manager compound_trajectory rrt mission)

# add_ros_executable(rrt_service src/nodes/rrt_service.cpp)
# target_link_libraries(rrt_service rrt kdtree-header-only ${OCTOMAP_LIBRARIES})

add_executable(object_map src/nodes/object_map.cpp)
add_ros_dependencies(object_map)
target_link_libraries(object_map ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
                      ${PCL_LIBRARIES})

add_ros_executable(get_octomap_from_server_and_write_to_bt_file
                   src/get_octomap_from_server_and_write_to_bt_file.cpp)
target_link_libraries(get_octomap_from_server_and_write_to_bt_file
                      ${OCTOMAP_LIBRARIES})
target_compile_definitions(get_octomap_from_server_and_write_to_bt_file
                           PUBLIC WRITE_BT_FILE)
add_ros_executable(
  compute_volume_of_occupied_voxels_in_octomap_and_write_to_stdout
  src/get_octomap_from_server_and_write_to_bt_file)
target_link_libraries(
  compute_volume_of_occupied_voxels_in_octomap_and_write_to_stdout
  ${OCTOMAP_LIBRARIES})
target_compile_definitions(
  compute_volume_of_occupied_voxels_in_octomap_and_write_to_stdout
  PUBLIC COMPUTE_VOLUME_AND_WRITE_TO_STDERR)

# target_link_libraries(object_map ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
# target_link_directories(object_map ${PCL_INCLUDE_DIRS})

# ##############################################################################
# TESTING
# ##############################################################################

# kdtree testing add_ros_executable(visualize_kdtree src/visualize_kdtree.cpp)
# add_ros_executable(linear_search_vs_kdtree test/linear_search_vs_kdtree.cpp)
# add_ros_executable(test_rrt test/rrt.test.cpp)
# add_ros_executable(test_controller src/test_controller.cpp)
# add_ros_executable(trajectory_follower src/trajectory_follower.cpp)
# add_ros_executable(spline_tester src/spline_tester.cpp)
# add_ros_executable(pid_controller src/pid_controller.cpp)
add_ros_executable(visualize_raycast test/visualize_raycast.cpp)

add_ros_executable(publish_traversed_path src/nodes/publish_traversed_path.cpp)

# rrt test
add_ros_executable(test_rrt test/rrt.test.cpp)
target_link_libraries(test_rrt rrt kdtree-header-only)
target_compile_definitions(test_rrt PUBLIC USE_KDTREE)

# fov visualization
add_ros_executable(visualize_fov test/visualize_fov.cpp)

# octomap testing
add_ros_executable(octomap_test test/octomap.test.cpp)
target_link_libraries(octomap_test rrt kdtree-header-only ${OCTOMAP_LIBRARIES})

add_ros_executable(test_nbv_service test/nbv_service.test.cpp)
add_ros_executable(test_rrt_service test/rrt_service.test.cpp)

add_ros_executable(rrt_service_vis_only src/nodes/rrt_service.cpp)
target_compile_definitions(rrt_service_vis_only
                           PUBLIC VISUALIZE_MARKERS_IN_RVIZ)
target_link_libraries(rrt_service_vis_only rrt kdtree-header-only
                      ${OCTOMAP_LIBRARIES})

# trajectory testing add_executable(trajectory_test test/trajectory.test.cpp)
# add_ros_dependencies(trajectory_test) target_link_libraries(trajectory_test
# ${catkin_LIBRARIES} compound_trajectory)
# target_compile_options(trajectory_test PUBLIC -O0 -g)

# ##############################################################################
# INSTALL
# ##############################################################################

catkin_install_python(PROGRAMS scripts/nodes/segmentation_service.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# scripts
function(install_script script)
  install(PROGRAMS ${script} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction(install_script)

install_script(scripts/make_px4_airsim.sh)
install_script(scripts/run_airsim.sh)
install_script(scripts/launch_airsim.sh)
install_script(scripts/make_px4_gazebo.sh)
