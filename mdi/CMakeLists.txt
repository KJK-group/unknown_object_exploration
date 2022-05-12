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
    PRIVATE -O0 -g
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
# add_ros_executable(spline_tester src/spline_tester.cpp)
add_ros_executable(pid_controller src/pid_controller.cpp)
add_ros_executable(publish_traversed_path src/publish_traversed_path.cpp)

add_ros_executable(visualize_raycast test/visualize_raycast.cpp)
add_ros_executable(visualize_fov test/visualize_fov.cpp)

# target_sources( spline_tester PUBLIC src/bezier_spline.cpp PUBLIC
# src/math.cpp)
add_library(bezier_spline SHARED src/bezier_spline.cpp)
target_sources(bezier_spline PUBLIC src/math.cpp)
add_library(rrt SHARED src/rrt/rrt.cpp)
target_link_libraries(rrt ${catkin_LIBRARIES} fmt::fmt-header-only
                      kdtree-header-only)

add_library(mission SHARED src/mission.cpp)
target_link_libraries(mission bezier_spline rrt)

# add_ros_executable(mission_manager src/mission_manager.cpp)
add_executable(mission_manager src/mission_manager.cpp)
add_dependencies(mission_manager ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_manager ${catkin_LIBRARIES} bezier_spline rrt
                      mission fmt::fmt-header-only)
target_compile_options(mission_manager PRIVATE -g)

target_sources(
  trajectory_follower
  PUBLIC src/bezier_spline.cpp
  PUBLIC src/math.cpp)

add_executable(test_rrt_service test/rrt_service.test.cpp)
add_dependencies(test_rrt_service ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_rrt_service ${catkin_LIBRARIES} rrt
                      fmt::fmt-header-only)
target_compile_options(test_rrt_service PRIVATE -O0 -g)

# target_sources( mission_manager # PUBLIC src/bezier_spline.cpp

# # PUBLIC src/rrt/rrt.cpp # PUBLIC src/mission.cpp

# )

target_sources(pid_controller PUBLIC src/transformlistener.cpp)

# add_library(rrt) target_sources(rrt PUBLIC src/rrt/rrt.cpp)
# target_link_libraries(rrt ${catkin_LIBRARIES} fmt::fmt-header-only
# kdtree-header-only)

function(add_ros_dependencies target)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
endfunction(add_ros_dependencies)

add_executable(rrt_service src/nodes/rrt_service.cpp)
add_ros_dependencies(rrt_service)
target_sources(rrt_service PRIVATE src/rrt/rrt.cpp)
# target_compile_options(rrt_service PRIVATE -O0 -g)
target_compile_options(rrt_service PRIVATE -O2 -g)
target_link_libraries(rrt_service ${catkin_LIBRARIES} fmt::fmt-header-only
                      kdtree-header-only ${OCTOMAP_LIBRARIES})

add_executable(octomap_test test/octomap.test.cpp)
add_ros_dependencies(octomap_test)
target_sources(octomap_test PRIVATE src/rrt/rrt.cpp)
target_compile_options(octomap_test PRIVATE -O2 -Wall)
target_link_libraries(octomap_test ${catkin_LIBRARIES} fmt::fmt-header-only
                      kdtree-header-only ${OCTOMAP_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################

install(PROGRAMS scripts/make_px4_airsim.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/run_airsim.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/launch_airsim.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/make_px4_gazebo.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
