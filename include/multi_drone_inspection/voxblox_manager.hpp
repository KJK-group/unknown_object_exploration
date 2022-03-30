#ifndef _MPI_VOXBLOX_MANAGER_HPP_
#define _MPI_VOXBLOX_MANAGER_HPP_

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>

#include <optional>

#include "multi_drone_inspection/voxelgrid.hpp"

namespace mpi {

class VoxbloxManager {
   private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    voxblox::EsdfServer esdf_server;

   public:
    VoxbloxManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~VoxbloxManager() = default;

    auto get_map_distance(const Eigen::Vector3d& position, bool interpolate = false)
        -> double const;
    auto get_voxel_status_at_position(const Eigen::Vector3d& position) -> VoxelStatus const;

   private:
    auto get_esdf_map_ptr() -> std::optional<decltype(esdf_server.getEsdfMapPtr())>;

};  // class VoxbloxManager

}  // namespace mpi

#endif  //_MPI_VOXBLOX_MANAGER_HPP_
