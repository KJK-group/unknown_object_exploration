cmake_minimum_required(VERSION 3.14)
project(
  mdi
  VERSION 0.1
  DESCRIPTION ""
  LANGUAGES CXX)

# GLOBAL CONFIGURATION ---------------------------------------------------------

# set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
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
# voxblox_ros)

include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

# find_package(PCL 1.0 REQUIRED COMPONENTS io)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# find_package(PCL 1.12 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})


file(GLOB PCL_LIBRARIES /usr/local/lib/libpcl_*)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.12)

INCLUDE_DIRECTORIES(${PROJECT_NAME} ${PCL_INCLUDE_DIRS})
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
# find_package(xacro REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# dependencies

# include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(deps/fmt)
add_subdirectory(libs/kdtree)
include_directories(include)

catkin_package()

# macro(append variable value) string(${variable} " ${value}") endmacro(append)

# set(COMMON_LIBS "") append(COMMON_LIBS ${catkin_LIBRARIES}) append(COMMON_LIBS
# fmt::fmt-header-only)

# # set(COMMON_LIBS ${catkin_LIBRARIES} fmt::fmt-header-only) message("common
# libraries: " ${COMMON_LIBS})

# ##############################################################################
# Build ##
# ##############################################################################

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

function(add_ros_dependencies target)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
endfunction(add_ros_dependencies)

# ##############################################################################
# Libraries ##
# ##############################################################################

# math library
add_library(math SHARED src/math.cpp)

# bezier library
add_library(bezier_spline SHARED src/bezier_spline.cpp)
target_link_libraries(bezier_spline math)

# linear trajectory library
add_library(linear_trajectory SHARED src/linear_trajectory.cpp)

# rrt library
add_library(rrt SHARED src/rrt.cpp)
target_link_libraries(rrt ${catkin_LIBRARIES} fmt::fmt-header-only
                      kdtree-header-only)

# compound trajectory library
add_library(compound_trajectory SHARED src/compound_trajectory.cpp)
target_link_libraries(compound_trajectory bezier_spline linear_trajectory rrt)

# mission library
add_library(mission SHARED src/mission.cpp)
target_link_libraries(mission compound_trajectory rrt)

# control library
add_library(control SHARED src/pid_controller.cpp)
target_sources(control PUBLIC src/transformlistener.cpp)

# ##############################################################################
# Executables ##
# ##############################################################################

# add_ros_executable(test_controller src/test_controller.cpp)
# add_ros_executable(trajectory_follower src/trajectory_follower.cpp)
# add_ros_executable(spline_tester src/spline_tester.cpp)
# add_ros_executable(pid_controller src/pid_controller.cpp)
add_ros_executable(publish_traversed_path src/nodes/publish_traversed_path.cpp)
add_ros_executable(visualize_raycast test/visualize_raycast.cpp)

add_executable(control_manager src/nodes/control_manager.cpp)
add_ros_dependencies(control_manager)
target_link_libraries(control_manager ${catkin_LIBRARIES} control)

add_executable(mission_manager src/nodes/mission_manager.cpp)
add_ros_dependencies(mission_manager)
target_link_libraries(mission_manager ${catkin_LIBRARIES} compound_trajectory
                      rrt mission fmt::fmt-header-only)
# target_compile_options(mission_manager PRIVATE -g)

# rrt service
add_executable(rrt_service src/nodes/rrt_service.cpp)
add_ros_dependencies(rrt_service)
# target_compile_options(rrt_service PRIVATE -O0 -g)
# target_compile_options(rrt_service PRIVATE -O2 -g)
target_link_libraries(rrt_service ${catkin_LIBRARIES} rrt fmt::fmt-header-only
                      kdtree-header-only ${OCTOMAP_LIBRARIES})

add_executable(object_map src/nodes/object_map.cpp)
add_ros_dependencies(object_map)
target_link_libraries(object_map ${catkin_LIBRARIES}
                      ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
# target_link_directories(object_map ${PCL_INCLUDE_DIRS})


# ##############################################################################
# Testing ##
# ##############################################################################

# kdtree testing add_ros_executable(visualize_kdtree src/visualize_kdtree.cpp)
# add_ros_executable(linear_search_vs_kdtree test/linear_search_vs_kdtree.cpp)
# add_ros_executable(test_rrt test/rrt.test.cpp)

# rrt test
add_executable(test_rrt test/rrt.test.cpp)
add_dependencies(test_rrt ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_rrt ${catkin_LIBRARIES} rrt fmt::fmt-header-only
                      kdtree-header-only)
# target_compile_options(test_rrt PRIVATE -g -O0 -Wall -Wextra -Wpedantic)
target_compile_options(test_rrt PRIVATE -O3 -finline-functions-called-once)
# target_compile_options(test_rrt PRIVATE -g -O0 -Wall -Wextra -Wpedantic)
# target_compile_definitions(test_rrt PUBLIC MEASURE_PERF)
target_compile_definitions(test_rrt PUBLIC USE_KDTREE)
# target_sources(visualize_kdtree PUBLIC src/kdtree3/kdtree3.cpp)
# target_compile_options(visualize_kdtree PRIVATE -g)

add_ros_executable(visualize_fov test/visualize_fov.cpp)

# octomap testing
add_executable(octomap_test test/octomap.test.cpp)
add_ros_dependencies(octomap_test)
target_compile_options(octomap_test PRIVATE -O2 -Wall)
target_link_libraries(octomap_test ${catkin_LIBRARIES} rrt fmt::fmt-header-only
                      kdtree-header-only ${OCTOMAP_LIBRARIES})

# trajectory testing add_executable(trajectory_test test/trajectory.test.cpp)
# add_ros_dependencies(trajectory_test) target_link_libraries(trajectory_test
# ${catkin_LIBRARIES} compound_trajectory)
# target_compile_options(trajectory_test PUBLIC -O0 -g)

# ##############################################################################
# Install ##
# ##############################################################################

# scripts
install(PROGRAMS scripts/make_px4_airsim.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/run_airsim.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/launch_airsim.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/make_px4_gazebo.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
