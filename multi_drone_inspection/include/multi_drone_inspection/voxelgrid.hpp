#ifndef _MULTI_DRONE_INSPECTION_VOXELGRID_HPP_
#define _MULTI_DRONE_INSPECTION_VOXELGRID_HPP_

#include <eigen3/Eigen/Dense>
#include <optional>

#include "multi_drone_inspection/voxelstatus.hpp"

namespace mdi {

using Eigen::Vector3f;

// * INTERFACE
class VoxelGrid {
   public:
    virtual ~VoxelGrid() {}  // ! IMPORTANT
    virtual auto get_voxel_status_at_position(const Vector3f& position) const&& -> VoxelStatus = 0;
    virtual auto raycast(const Vector3f& position, const Vector3f& direction,
                         float distance) const&& -> std::optional<Vector3f> = 0;

    virtual auto raycast(const Vector3f& position,
                         const Vector3f& direction) const&& -> std::optional<Vector3f> = 0;

    virtual auto get_voxel_sixe() const&& -> float = 0;
    virtual auto get_euclidian_distance_to_closest_occupied_voxel_from(
        const Vector3f& position) const&& -> std::optional<float> = 0;
    virtual auto get_position_of_closest_occupied_voxel_from(
        const Vector3f& position) const&& -> std::optional<Vector3f> = 0;
};  // class VoxelGrid

}  // namespace mdi

#endif  // _MULTI_DRONE_INSPECTION_VOXELGRID_HPP_
