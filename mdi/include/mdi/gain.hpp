#pragma once

#include <cmath>

#include "mdi/VoxelStatus.hpp"
#include "mdi/bbx.hpp"
#include "mdi/common_types.hpp"
#include "mdi/fov.hpp"
#include "mdi/octomap.hpp"
#include "octomap.hpp"
#include "voxelstatus.hpp"

namespace mdi {

auto gain_of_fov(const types::FoV& fov, const mdi::Octomap& octomap, const double weight_free,
                 const double weight_occupied, const double weight_unknown) -> double {
    using namespace mdi::types;
    using mdi::VoxelStatus;
    using point = mdi::Octomap::point_type;

    const auto bbx = mdi::compute_bbx(fov);

    const point origin = [&] {
        const auto pos = fov.pose().position;
        return point{pos.x(), pos.y(), pos.z()};
    }();

    const auto visible = [&](const vec3& v) {
        auto opt = octomap.raycast(origin, point{v.x(), v.y(), v.z()});
        return ! opt.has_value();
    };

    double gain = 0;

    octomap.iterate_over_bbx(bbx, [&](const auto& pt, VoxelStatus vs) {
        const vec3 v = vec3{pt.x(), pt.y(), pt.z()};
        if (fov.inside_fov(v) && visible(v)) {
            switch (vs) {
                case VoxelStatus::Free:
                    gain += weight_free;
                    break;
                case VoxelStatus::Occupied:
                    gain += weight_occupied;
                    break;

                case VoxelStatus::Unknown:
                    gain += weight_unknown;
                    break;
            }
        }
    });

    // scale with volume
    gain *= std::pow(octomap.resolution(), 3.0);

    return gain;
}

}  // namespace mdi
