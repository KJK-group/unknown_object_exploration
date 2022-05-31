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
    std::size_t n_inside_fov;
    std::size_t n_visible;
    std::size_t n_not_visible;
    std::size_t n_free_voxels;
    std::size_t n_occupied_voxels;
    std::size_t n_unknown_voxels;
    std::size_t n_total;
};  // FoVGainMetric

auto yaml(const FoVGainMetric metric, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto tab2 = tab + tab;
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };
    return line("FoVGainMetric:") + line(tab + "voxels: ") +
           line(tab2 + "free: " + std::to_string(metric.n_free_voxels)) +
           line(tab2 + "occupied: " + std::to_string(metric.n_occupied_voxels)) +
           line(tab2 + "unknown: " + std::to_string(metric.n_unknown_voxels)) +
           line(tab + "gains:") + line(tab2 + "free: " + std::to_string(metric.gain_free)) +
           line(tab2 + "occupied: " + std::to_string(metric.gain_occupied)) +
           line(tab2 + "unknown: " + std::to_string(metric.gain_unknown)) +
           line(tab2 + "distance: " + std::to_string(metric.gain_distance)) +
           line(tab2 + "not_visible: " + std::to_string(metric.gain_not_visible)) +
           line(tab2 + "total: " + std::to_string(metric.gain_total)) + line(tab + "fov:") +
           line(tab2 + "total: " + std::to_string(metric.n_total)) +
           line(tab2 + "visible: " + std::to_string(metric.n_visible)) +
           line(tab2 + "not_visible: " + std::to_string(metric.n_not_visible)) +
           line(tab2 + "inside: " + std::to_string(metric.n_inside_fov)) +
           line(tab2 +
                "included: " + std::to_string(std::min(metric.n_inside_fov, metric.n_visible)));
}

auto to_ros_msg(const FoVGainMetric& m) -> mdi_msgs::FoVGainMetric {
    auto msg = mdi_msgs::FoVGainMetric{};

    msg.gain_free = m.gain_free;
    msg.gain_unknown = m.gain_unknown;
    msg.gain_occupied = m.gain_occupied;
    msg.gain_distance = m.gain_distance;
    msg.gain_total = m.gain_total;

    msg.n_inside_fov = m.n_inside_fov;
    msg.n_visible = m.n_visible;
    msg.n_not_visible = m.n_not_visible;
    msg.n_free_voxels = m.n_free_voxels;
    msg.n_occupied_voxels = m.n_occupied_voxels;
    msg.n_unknown_voxels = m.n_unknown_voxels;
    msg.n_total = m.n_total;

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

    octomap.iterate_over_bbx(bbx, [&](const auto& pt, VoxelStatus vs) {
        const vec3 v = vec3{pt.x(), pt.y(), pt.z()};
        const double distance_to_target = distance_tf((fov.target() - v).norm());

        m.n_total += 1;

        if (fov.inside_fov(v)) {
            m.n_inside_fov += 1;
            if (visible(v)) {
                m.n_visible += 1;
                switch (vs) {
                    case VoxelStatus::Free:
                        m.gain_free += weight_free;
                        m.n_free_voxels += 1;
                        break;
                    case VoxelStatus::Occupied:
                        m.gain_occupied += weight_occupied;
                        m.n_occupied_voxels += 1;
                        break;
                    case VoxelStatus::Unknown:
                        m.gain_unknown += weight_unknown;
                        m.gain_distance +=
                            weight_distance_to_target * (distance_total / distance_to_target);
                        m.n_unknown_voxels += 1;
                        break;
                }
            } else {
                m.n_not_visible += 1;
                m.gain_not_visible += weight_not_visible;
            }
        }
    });

    // scale with volume
    const double resolution_scale_factor = std::pow(octomap.resolution(), 3.0);
    m.gain_free *= resolution_scale_factor;
    m.gain_occupied *= resolution_scale_factor;
    m.gain_unknown *= resolution_scale_factor;
    m.gain_distance *= resolution_scale_factor;
    m.gain_not_visible *= resolution_scale_factor;

    m.gain_total =
        m.gain_free + m.gain_occupied + m.gain_unknown + m.gain_distance + m.gain_not_visible;

    return m;
}

}  // namespace mdi
