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

auto gain_of_fov(
    const types::FoV& fov, const mdi::Octomap& octomap, const double weight_free,
    const double weight_occupied, const double weight_unknown,
    const double weight_distance_to_target, types::vec3 root,
    std::function<double(double)> distance_tf,
    std::function<void(const types::vec3&, const types::vec3&, float, const bool)> visit_cb)
    -> double {
    using namespace mdi::types;
    using mdi::VoxelStatus;
    using point = mdi::Octomap::point_type;

    const auto bbx = mdi::compute_bbx(fov);
    // std::cout << "\nweight_free: " << weight_free << "\nweight_occupied: " << weight_occupied
    //           << "\nweight_unknown: " << weight_unknown
    //           << "\nweight_distance: " << weight_distance_to_target << std::endl;

    const point origin = [&] {
        const auto pos = fov.pose().position;
        return point{pos.x(), pos.y(), pos.z()};
    }();

    const auto visible = [&](const vec3& v) -> bool {
        auto voxel = octomap.raycast(origin, point{v.x(), v.y(), v.z()}, false);

        const auto match = [&](auto&& x) {
            auto visitor = Overload{
                [](Free) { return true; },
                [](Unknown) { return false; },
                [](Occupied) { return false; },
            };

            return std::visit(visitor, x);
        };

        const bool is_visible = match(voxel);
        const auto dir = (v - fov.pose().position);
        visit_cb(fov.pose().position, dir.normalized(), dir.norm(), is_visible);
        return is_visible;
    };

    double gain = 0;
    int free_counter = 0;
    int occupied_counter = 0;
    int unknown_counter = 0;
    // int inside_fov_counter = 0;
    // int visible_counter = 0;
    const double distance_total = distance_tf((fov.target() - root).norm());
    // std::cout << "distance_total: " << distance_total << std::endl;

    auto gain_free = 0.0;
    auto gain_occupied = 0.0;
    auto gain_unknown = 0.0;
    auto gain_distance = 0.0;

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
                    gain_free += weight_free;
                    free_counter++;
                    break;
                case VoxelStatus::Occupied:
                    gain_occupied += weight_occupied;
                    occupied_counter++;
                    break;
                case VoxelStatus::Unknown:
                    gain_unknown += weight_unknown;
                    gain_distance +=
                        weight_distance_to_target * (1 - (distance_to_target / distance_total));
                    unknown_counter++;
                    break;
            }
        }
    });

    std::cout << "\n\nfree:\t" << free_counter << "\twith gain:\t" << gain_free << "\noccupied:\t"
              << occupied_counter << "\twith gain:\t" << gain_occupied << "\nunknown:\t"
              << unknown_counter << "\twith gain:\t" << gain_unknown << "\nand distance gain:\t"
              << gain_distance << std::endl;

    // ROS_INFO_STREAM("\ninside fov: " << inside_fov_counter << "\nvisible: " << visible_counter
    //                                  << "\nfree: " << free_counter << "\noccupied: "
    //                                  << occupied_counter << "\nunknown: " << unknown_counter);

    gain = gain_free + gain_occupied + gain_unknown + gain_distance;
    // scale with volume
    gain *= std::pow(octomap.resolution(), 3.0);

    return gain;
}

}  // namespace mdi
