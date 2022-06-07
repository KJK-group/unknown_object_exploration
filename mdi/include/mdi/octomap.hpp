#pragma once

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <functional>
#include <utility>

#include "mdi/bbx.hpp"
#include "mdi/common_headers.hpp"
#include "mdi/voxelstatus.hpp"
#include "octomap/OcTreeKey.h"
// #include <pcl-1.12/pcl/impl/point_types.hpp>
#include "mdi/common_types.hpp"
#include "pcl/impl/point_types.hpp"
#include "voxelstatus.hpp"

namespace mdi {
constexpr auto DEFAULT_MIN_THRESH_PROB = 0.1;
constexpr auto DEFAULT_MAX_THRESH_PROB = 0.5;

class Octomap final {
   public:
    // PUBLIC TYPES
    // ---------------------------------------------------------------------------------------------
    using point_type = octomap::point3d;
    using octree_type = octomap::OcTree;
    using node_type = octree_type::NodeType;

    // struct BBX {
    //     point_type center;
    //     float width, height, depth;
    //     [[nodiscard]] auto min() const -> point_type {
    //         return {center.x() - width / 2, center.z() - height / 2, center.z() - depth / 2};
    //     }
    //     [[nodiscard]] auto max() const -> point_type {
    //         return {center.x() + width / 2, center.z() + height / 2, center.z() + depth / 2};
    //     }
    // };  // BBX

   public:
    // FRIENDS
    // ---------------------------------------------------------------------------------------------
    // friend std::ostream& operator<<(std::ostream& os, const Octomap& octomap);
   public:
    // CONSTRUCTORS
    // ---------------------------------------------------------------------------------------------
    Octomap(double resolution) : octree_(resolution) {}

    // Octomap(double resolution, double thresh_prob_min = DEFAULT_MIN_THRESH_PROB,
    //         double thresh_prob_max = DEFAULT_MAX_THRESH_PROB)
    //     : octree_(resolution) {
    //     octree_.setClampingThresMax(thresh_prob_max);
    //     octree_.setClampingThresMin(thresh_prob_min);
    // }
    // Octomap(octree_type&& tree) : octree_(tree) {}
    Octomap(const Octomap& octree) = delete;
    Octomap(const octomap_msgs::Octomap& map) : octree_{map.resolution} {
        if (! map.binary) {
            throw std::invalid_argument("octomap should be in binary format");
        }
        const auto& data = map.data;

        if (data.size() <= 0) {
            throw std::invalid_argument("the msg has no data");
        }

        auto data_stream = std::stringstream{};
        data_stream.write((const char*)&data[0], data.size());
        octree_.readBinaryData(data_stream);
        std::cout << "read binary data into octree" << std::endl;
    }

    /**
     * @brief factory method for creating octree from octomap_msgs::Octomap.
     * The octree will be created with the resolution in msg->resolution.
     * @param msg
     * @return Octomap
     */
    // static auto from_octomap_msg(const octomap_msgs::Octomap& map) -> std::unique_ptr<Octomap> {
    //     return {map};  // call private constructor
    // }

    // Octomap(OcTree&& tree) : octree_(std:Const:move(tree)) {}

   public
       :  // PUBLIC INTERFACE
          // ----------------------------------------------------------------------------------------
    auto raycast_in_direction(const point_type& origin, const point_type& direction,
                              double max_range, bool ignore_unknown_voxels = false) const -> Voxel {
        return raycast_(origin, direction, max_range, ignore_unknown_voxels);
    }

    auto raycast(const point_type& origin, const point_type& end,
                 bool ignore_unknown_voxels = false) const -> Voxel {
        const auto direction = end - origin;
        const auto max_range = direction.norm();
        return raycast_(origin, direction, max_range, ignore_unknown_voxels);
    }

    auto get_voxel_status_at_point(const point_type& point) const -> VoxelStatus {
        if (const auto opt = search_for_node_(point)) {
            const auto node_ptr = opt.value();
            return octree_.isNodeOccupied(node_ptr) ? VoxelStatus::Occupied : VoxelStatus::Free;
        }

        return VoxelStatus::Unknown;
    }

    auto compute_total_volume_voxels_with_status(VoxelStatus status) -> double {
        // calculate the total volume of the received object map
        auto volume_total = 0.0;
        for (auto it = octree().begin_tree(), end = octree().end_tree(); it != end; ++it) {
            const auto key = it.getKey();
            const auto vs = get_voxelstatus_at_node_using_key_(key);
            if (vs == status) {
                const auto size = it.getSize();
                const auto volume = std::pow(size, 3);
                volume_total += volume;
            }
        }

        return volume_total;
    }

    auto compute_total_volume_of_occupied_voxels() -> double {
        return compute_total_volume_voxels_with_status(VoxelStatus::Occupied);
    }

    /*    auto get_closest_intersection_point(const point_type& origin, const point_type& direction,
                                           double delta = 0.0) const -> std::optional<point_type> {
           if (const auto opt = raycast_(origin, direction)) {
               const auto center_pt_of_hit_voxel = opt.value();
               auto intersection = point_type{0, 0, 0};

               const auto intersected = octree_.getRayIntersection(
                   origin, direction, center_pt_of_hit_voxel, intersection, delta);
               if (intersected) {
                   return intersection;
               }
           }

           return std::nullopt;
       } */

    // // TODO:
    // auto get_closest_occupied_voxel(const point_type& point) const -> std::optional<point_type> {
    //     return std::nullopt;
    // }

    auto resolution() const -> double { return octree_.getResolution(); }
    // TODO:
    auto write(const std::filesystem::path& path, bool overwrite = false) const -> bool {
        if (! overwrite && std::filesystem::is_regular_file(path)) {
            std::cerr << "";
            return false;
        }

        auto s = std::ofstream(path);
        octree_.writeData(s);
    }

    using bbx_iterator_cb =
        std::function<void(const point_type& pt, const double size, const VoxelStatus vs)>;

    auto iterate_over_bbx(const mdi::types::BBX& bbx, bbx_iterator_cb cb) const -> void {
        const auto [min, max] = [&] {
            const auto min = bbx.min();
            const auto max = bbx.max();
            const point_type start = {min.x(), min.y(), min.z()};
            const point_type stop = {max.x(), max.y(), max.z()};
            return std::make_pair(start, stop);
        }();

        for (auto it = octree_.begin_leafs_bbx(min, max), end = octree_.end_leafs_bbx(); it != end;
             ++it) {
            const auto voxel_center = it.getCoordinate();
            const auto size = it.getSize();
            // ROS_INFO_STREAM("getSize() -> " << size);
            const auto key = it.getKey();

            cb(voxel_center, size, get_voxelstatus_at_node_using_key_(key));
        }

        octomap::point3d_list voxel_centers{};

        octree_.getUnknownLeafCenters(voxel_centers, min, max);
        for (auto voxel_center : voxel_centers) {
            // TODO: check if resolution() is correct here
            // ROS_INFO_STREAM("unknown so using DEFAULT resolution: " << resolution());

            cb(voxel_center, resolution(), VoxelStatus::Unknown);
        }
    }

    /**
     * @brief returns a mutable reference to the underlying OcTree
     * @return const OcTree&
     */
    auto octree() -> octree_type& { return octree_; }

   private:
    // ---------------------------------------------------------------------------------------------
    octree_type octree_;

    // PRIVATE METHODS
    // ---------------------------------------------------------------------------------------------
    // Octomap(const octomap_msgs::Octomap& map) : octree_{map.resolution} {
    //     const auto& data = map.data;
    //     auto data_stream = std::stringstream{};
    //     data_stream.write((const char*)&data[0], data.size());
    //     octree_.readBinaryData(data_stream);
    // }

    auto raycast_(const point_type& origin, const point_type& direction, double max_range = -1,
                  bool ignore_unknown_voxels = false) const -> Voxel {
        auto end = point_type{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max()};
        const auto intersected_occupied_voxel =
            octree_.castRay(origin, direction, end, ignore_unknown_voxels, max_range);
        if (intersected_occupied_voxel) {
            return Occupied{end};
        }
        if ((end - origin).norm() < max_range) {
            return Unknown{end};
        }

        return Free{};
    }

    auto get_voxelstatus_at_node_using_key_(const octomap::OcTreeKey key) const -> VoxelStatus {
        if (const auto opt = search_for_node_using_key_(key)) {
            const auto node_ptr = opt.value();
            return octree_.isNodeOccupied(node_ptr) ? VoxelStatus::Occupied : VoxelStatus::Free;
        }

        return VoxelStatus::Unknown;
    }

    auto search_for_node_using_key_(const octomap::OcTreeKey key, unsigned int depth = 0) const
        -> std::optional<node_type*> {
        const auto node_ptr = octree_.search(key, depth);
        if (node_ptr != nullptr) {
            return node_ptr;
        }

        return std::nullopt;
    }

    auto search_for_node_(const point_type& point, unsigned int depth = 0) const
        -> std::optional<node_type*> {
        const auto node_ptr = octree_.search(point, depth);
        if (node_ptr != nullptr) {
            return node_ptr;
        }

        return std::nullopt;
    }
};  // namespace mdi

// std::ostream& operator<<(std::ostream& os, const Octomap& octomap) {
// os << "octomap:\n";
// os << "  resolution: " << octomap.resolution() << '\n';
// os << "  bbx:" << '\n';
// os << "    max:" << '\n';
// os << "      x:" << '\n';
// os << "      y:" << '\n';
// os << "      z:" << '\n';
// os << "    min:" << '\n';
// os << "      x:" << '\n';
// os << "      y:" << '\n';
// os << "      z:" << '\n';
//
// return os;
// }

}  // namespace mdi
