#pragma once

#include <cmath>
#include <functional>
#include <variant>

#include "mdi/bbx.hpp"
#include "mdi/common_types.hpp"
#include "mdi/fov.hpp"
#include "mdi/octomap.hpp"
#include "mdi/voxelstatus.hpp"

namespace mdi {

auto gain_of_fov(const types::FoV& fov, const mdi::Octomap& octomap, const double weight_free,
                 const double weight_occupied, const double weight_unknown,
                 const double weight_distance_to_target, types::vec3 root,
                 std::function<double(double)> distance_tf) -> double {
    using namespace mdi::types;
    using mdi::VoxelStatus;
    using point = mdi::Octomap::point_type;

    const auto bbx = mdi::compute_bbx(fov);

    const point origin = [&] {
        const auto pos = fov.pose().position;
        return point{pos.x(), pos.y(), pos.z()};
    }();

    const auto visible = [&](const vec3& v) -> bool {
        auto /* opt */ voxel = octomap.raycast(origin, point{v.x(), v.y(), v.z()}, false);
        // return ! opt.has_value();

        // auto match = Overload{
        //     [](Free _) { return true; },
        //     [](Unknown _) { return false; },
        //     [](Occupied _) { return false; },
        // };

        const auto match = [&](auto&& x) {
            auto visitor = Overload{
                [](Free) { return true; },
                [](Unknown) { return false; },
                [](Occupied) { return false; },
            };

            return std::visit(visitor, x);
        };

        // return std::visit(match, voxel);
        return match(voxel);
    };

    double gain = 0;
    // int free_counter = 0;
    // int occupied_counter = 0;
    // int unknown_counter = 0;
    // int inside_fov_counter = 0;
    // int visible_counter = 0;
    const double distance_total = distance_tf((fov.target() - root).norm());
    // std::cout << "distance_total: " << distance_total << std::endl;

    octomap.iterate_over_bbx(bbx, [&](const auto& pt, VoxelStatus vs) {
        const vec3 v = vec3{pt.x(), pt.y(), pt.z()};
        const double distance_to_target = distance_tf((fov.target() - v).norm());
        // std::cout << "distance_to_target: " << distance_to_target << std::endl;

        // if (fov.inside_fov(v)) {
        //     inside_fov_counter++;
        // }
        // if (visible(v)) {
        //     visible_counter++;
        // }

        if (fov.inside_fov(v) && visible(v)) {
            switch (vs) {
                case VoxelStatus::Free:
                    gain += weight_free;
                    // free_counter++;
                    break;
                case VoxelStatus::Occupied:
                    gain += weight_occupied;
                    // occupied_counter++;
                    break;
                case VoxelStatus::Unknown:
                    gain += weight_unknown +
                            weight_distance_to_target * (distance_total / distance_to_target);
                    // unknown_counter++;
                    break;
            }
        }
    });

    // ROS_INFO_STREAM("\ninside fov: " << inside_fov_counter << "\nvisible: " << visible_counter
    //                                  << "\nfree: " << free_counter << "\noccupied: "
    //                                  << occupied_counter << "\nunknown: " << unknown_counter);

    // scale with volume
    gain *= std::pow(octomap.resolution(), 3.0);

    return gain;
}

}  // namespace mdi
