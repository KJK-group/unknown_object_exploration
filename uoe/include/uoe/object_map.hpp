#pragma once

#include <cmath>
#include <vector>

#include "common_types.hpp"
#include "octomap.hpp"
#include "utils/eigen.hpp"

namespace uoe::types::object {
constexpr auto DEFAULT_MIN_THRESH_PROB = 0.1;
constexpr auto DEFAULT_MAX_THRESH_PROB = 0.5;

class ObjectMap {
   private:
    std::vector<vec3> point_cloud_;
    Octomap map_;

   public:
    ObjectMap();
    ObjectMap(double resolution, double thresh_prob_min = DEFAULT_MIN_THRESH_PROB,
              double thresh_prob_max = DEFAULT_MAX_THRESH_PROB)
        : map_({resolution}) {
        map_.octree().setClampingThresMax(thresh_prob_max);
        map_.octree().setClampingThresMin(thresh_prob_min);
    }
    [[nodiscard]] auto resolution() const -> double { return map_.resolution(); }
    auto insert_points(std::vector<vec3> points) -> void {}
    auto is_obejct(vec3 position) -> bool {
        auto status = map_.get_voxel_status_at_point({position.x(), position.y(), position.z()});
        return status == VoxelStatus::Occupied ? true : false;
    }
};
}  // namespace uoe::types::object