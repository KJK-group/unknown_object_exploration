#ifndef _MPI_VOXBLOX_MANAGER_HPP_
#define _MPI_VOXBLOX_MANAGER_HPP_

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>

#include <eigen3/Eigen/Dense>
#include <optional>

#include "multi_drone_inspection/voxelgrid.hpp"
#include "multi_drone_inspection/voxelstatus.hpp"

namespace mpi {

class VoxbloxManager : public VoxelGrid {
   private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    voxblox::EsdfServer esdf_server;

   public:
    VoxbloxManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~VoxbloxManager() {}

    // interface start
    auto get_voxel_status_at_position(const Vector3f& position) const -> VoxelStatus override final;
    auto raycast(const Vector3f& position, const Vector3f& direction, float distance) const
        -> std::optional<Vector3f> override final;

    auto raycast(const Vector3f& position, const Vector3f& direction) const
        -> std::optional<Vector3f> override final;

    auto get_voxel_sixe() const -> float override final;
    auto get_euclidian_distance_to_closest_occupied_voxel_from(const Vector3f& position) const
        -> std::optional<float> override final;
    auto get_position_of_closest_occupied_voxel_from(const Vector3f& position) const
        -> std::optional<Vector3f> override final;
    // interface end

    auto get_map_distance(const Eigen::Vector3d& position, bool interpolate = false)
        -> double const;
    // auto get_voxel_status_at_position(const Eigen::Vector3d& position) -> VoxelStatus const;

   private:
    auto get_esdf_map_ptr() -> std::optional<decltype(esdf_server.getEsdfMapPtr())>;

};  // class VoxbloxManager

}  // namespace mpi

#endif  //_MPI_VOXBLOX_MANAGER_HPP_
