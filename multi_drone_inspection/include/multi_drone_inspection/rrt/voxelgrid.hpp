#pragma once

#include <eigen3/Eigen/Dense>
#include <optional>

using vec3 = Eigen::Vector3f;

template <typename VoxelGrid>
auto raycast(VoxelGrid& grid) -> bool {
    return true;
}

struct OctoMap {};  // OctoMap
struct Voxblox {};  // Voxblox

template <>
auto raycast<OctoMap>(OctoMap& grid, const vec3& origin, const vec3& direction, bool ignore_unknown_voxels = false,
                      double max_range = 1.0) -> std::optional<vec3> {
    // https://octomap.github.io/octomap/doc/classoctomap_1_1OccupancyOcTreeBase.html#a6371096f480cf835765286bfffd58708
    auto end = vec3{0, 0, 0};
    const auto intersected_occupied_voxel = grid.castRay(origin, direction, end, ignore_unknown_voxels, max_range);
    if (intersected_occupied_voxel) {
        return end;
    }

    return std::nullopt;
}

template <>
auto raycast<Voxblox>(Voxblox& grid, const vec3& origin, const vec3& direction, bool ignore_unknown_voxels = false,
                      double max_range = 1.0) -> std::optional<vec3> {
    // This involves doing a raycast from view point to voxel to test.
    // Let's get the global voxel coordinates of both.
    const static double voxel_size = tsdf_layer_->voxel_size();
    const static double voxel_size_inv = 1.0 / voxel_size;

    const voxblox::Point origin_scaled = origin.cast<voxblox::FloatingPoint>() * voxel_size_inv;
    const voxblox::Point end_scaled = end.cast<voxblox::FloatingPoint>() * voxel_size_inv;
    auto global_voxel_indices = voxblox::LongIndexVector{};
    voxblox::rayCast(origin_scaled, end_scaled, global_voxel_indices);
    for (const auto& global_index : global_voxel_indices) {
        auto* voxel = tsdf_layer_->getVoxelPtrByGlobalIndex(global_index);
        const auto voxel_is_unknown = voxel == nullptr || voxel->weight < 1e-6;
        if (voxel_is_unknown) {
            if (ignore_unknown_voxels) {
                // unknown voxel
            }
        } else if (voxel->distance <= 0.0) {
            // occupied
        }
    }
    return std::nullopt;
}

VoxelStatus VoxbloxManager::getVisibility(const Point& view_point, const Point& voxel_to_test,
                                          bool stop_at_unknown_voxel) const {
    // This involves doing a raycast from view point to voxel to test.
    // Let's get the global voxel coordinates of both.
    const static double voxel_size = tsdf_layer_->voxel_size();
    const static double voxel_size_inv = 1.0 / voxel_size;

    const voxblox::Point start_scaled = view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;
    const voxblox::Point end_scaled = voxel_to_test.cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector global_voxel_indices;
    voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
    // Iterate over the ray.
    for (const voxblox::GlobalIndex& global_index : global_voxel_indices) {
        voxblox::TsdfVoxel* voxel = tsdf_layer_->getVoxelPtrByGlobalIndex(global_index);
        if (voxel == nullptr || voxel->weight < 1e-6) {
            if (stop_at_unknown_voxel) {
                return VoxelStatus::kUnknown;
            }
        } else if (voxel->distance <= 0.0) {
            return VoxelStatus::kOccupied;
        }
    }
    return VoxelStatus::kFree;
}
