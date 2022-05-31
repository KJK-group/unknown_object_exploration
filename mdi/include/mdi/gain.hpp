#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <variant>

#include "mdi/bbx.hpp"
#include "mdi/common_types.hpp"
#include "mdi/fov.hpp"
#include "mdi/octomap.hpp"
#include "mdi/voxelstatus.hpp"
#include "mdi_msgs/FoVGainMetric.h"

namespace mdi {

struct FoVGainMetric {
    double gain_free, gain_unknown, gain_occupied, gain_distance, gain_not_visible;
    double gain_total;
    double v_inside_fov;
    double v_visible;
    double v_not_visible;
    double v_free_voxels;
    double v_occupied_voxels;
    double v_unknown_voxels;
    double v_total;
};  // FoVGainMetric

auto yaml(const FoVGainMetric metric, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto tab2 = tab + tab;
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };
    return line("FoVGainMetric:") + line(tab + "voxels: ") +
           line(tab2 + "free: " + std::to_string(metric.v_free_voxels)) +
           line(tab2 + "occupied: " + std::to_string(metric.v_occupied_voxels)) +
           line(tab2 + "unknown: " + std::to_string(metric.v_unknown_voxels)) +
           line(tab + "gains:") + line(tab2 + "free: " + std::to_string(metric.gain_free)) +
           line(tab2 + "occupied: " + std::to_string(metric.gain_occupied)) +
           line(tab2 + "unknown: " + std::to_string(metric.gain_unknown)) +
           line(tab2 + "distance: " + std::to_string(metric.gain_distance)) +
           line(tab2 + "not_visible: " + std::to_string(metric.gain_not_visible)) +
           line(tab2 + "total: " + std::to_string(metric.gain_total)) + line(tab + "fov:") +
           line(tab2 + "total: " + std::to_string(metric.v_total)) +
           line(tab2 + "visible: " + std::to_string(metric.v_visible)) +
           line(tab2 + "not_visible: " + std::to_string(metric.v_not_visible)) +
           line(tab2 + "inside: " + std::to_string(metric.v_inside_fov));
}

auto to_ros_msg(const FoVGainMetric& m) -> mdi_msgs::FoVGainMetric {
    auto msg = mdi_msgs::FoVGainMetric{};

    msg.gain.free = m.gain_free;
    msg.gain.unknown = m.gain_unknown;
    msg.gain.occupied = m.gain_occupied;
    msg.gain.not_visible = m.gain_not_visible;
    msg.gain.distance = m.gain_distance;
    msg.gain.total = m.gain_total;

    msg.fov.inside = m.v_inside_fov;
    msg.fov.visible = m.v_visible;
    msg.fov.not_visible = m.v_not_visible;
    msg.fov.total = m.v_total;
    msg.voxels.free = m.v_free_voxels;
    msg.voxels.occupied = m.v_occupied_voxels;
    msg.voxels.unknown = m.v_unknown_voxels;

    return msg;
}

auto gain_of_fov(
    const types::FoV& fov, const mdi::Octomap& octomap, const double weight_free,
    const double weight_occupied, const double weight_unknown,
    const double weight_distance_to_target, const double weight_not_visible, types::vec3 root,
    std::function<double(double)> distance_tf,
    std::function<void(const types::vec3&, const types::vec3&, float, const bool)> visit_cb)
    -> FoVGainMetric {
    using namespace mdi::types;
    using mdi::VoxelStatus;
    using point = mdi::Octomap::point_type;

    const auto bbx = mdi::compute_bbx(fov);

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

    auto m = FoVGainMetric{};

    const double distance_total = distance_tf((fov.target() - root).norm());

    octomap.iterate_over_bbx(bbx, [&](const auto& pt, const double size, VoxelStatus vs) {
        const vec3 v = vec3{pt.x(), pt.y(), pt.z()};
        const double distance_to_target = distance_tf((fov.target() - v).norm());

        // m.v_total += volume_scale_factor;

        if (fov.inside_fov(v)) {
            const double voxel_volume = std::pow(size, 3.0);
            m.v_inside_fov += voxel_volume;
            if (visible(v)) {
                m.v_visible += voxel_volume;
                switch (vs) {
                    case VoxelStatus::Free:
                        m.gain_free += weight_free * voxel_volume;
                        m.v_free_voxels += voxel_volume;
                        break;
                    case VoxelStatus::Occupied:
                        m.gain_occupied += weight_occupied * voxel_volume;
                        m.v_occupied_voxels += voxel_volume;
                        break;
                    case VoxelStatus::Unknown:
                        m.gain_unknown += weight_unknown * voxel_volume *
                                          weight_distance_to_target *
                                          (1 - (std::pow(distance_to_target, 2) / distance_total));
                        m.gain_distance += weight_distance_to_target *
                                           (1 - (std::pow(distance_to_target, 2) / distance_total));
                        m.v_unknown_voxels += voxel_volume;
                        break;
                }
            } else {
                m.v_not_visible += voxel_volume;
                m.gain_not_visible += weight_not_visible * voxel_volume;
            }
        }
    });

    // scale with volume
    // m.gain_free *= resolution_scale_factor;
    // m.gain_occupied *= resolution_scale_factor;
    // m.gain_unknown *= resolution_scale_factor;
    // m.gain_distance *= resolution_scale_factor;
    // m.gain_not_visible *= resolution_scale_factor;

    m.gain_total =
        (m.gain_free + m.gain_occupied + m.gain_unknown + m.gain_not_visible) / m.v_inside_fov;

    return m;
}

}  // namespace mdi
