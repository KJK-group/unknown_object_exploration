#include <geometry_msgs/Point.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <istream>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "octomap/octomap_types.h"

using namespace std;
using namespace octomap;
enum class VoxelStatus {
    free,
    occupied,
    unknown,
};

class OcTreeWrapper final {
   public:  // public types
    using point_type = octomap::point3d;
    using node_type = octomap::OcTree::NodeType;

    struct BBX {
        point3d center;
        float width, height, depth;
        [[nodiscard]] auto min() const -> point3d {
            return {center.x() - width / 2, center.z() - height / 2, center.z() - depth / 2};
        }
        [[nodiscard]] auto max() const -> point3d {
            return {center.x() + width / 2, center.z() + height / 2, center.z() + depth / 2};
        }
    };  // BBX

   public:  // constructors
    OcTreeWrapper(double resolution) : octree_(resolution) {}
    OcTreeWrapper(OcTree&& tree) : octree_(tree) {}

    /**
     * @brief factory method for creating octree from octomap_msgs::Octomap.
     * The octree will be created with the resolution in msg->resolution.
     * @param msg
     * @return OcTreeWrapper
     */
    static auto from_octomap_msg(const octomap_msgs::Octomap::Ptr& msg) -> OcTreeWrapper {
        if (! msg->binary) {
            throw std::invalid_argument("octomap should be in binary format");
        }
        const auto& data = msg->data;

        if (data.size() <= 0) {
            throw std::invalid_argument("the msg has no data");
        }

        return {msg};  // call private constructor
    }

    // OcTreeWrapper(OcTree&& tree) : octree_(std::move(tree)) {}
    OcTreeWrapper(const OcTreeWrapper& octree) = delete;

   public:  // public interface
    auto raycast(const point_type& origin, const point_type& direction, double max_range,
                 bool ignore_unknown_voxels = false) const -> std::optional<point_type> {
        return raycast_(origin, direction, max_range, ignore_unknown_voxels);
    }

    auto get_voxel_status_at_point(const point_type& point) const -> VoxelStatus {
        if (const auto opt = search_for_node_(point)) {
            const auto node_ptr = opt.value();
            return octree_.isNodeOccupied(node_ptr) ? VoxelStatus::occupied : VoxelStatus::free;
        }

        return VoxelStatus::unknown;
    }

    auto get_closest_intersection_point(const point_type& origin, const point_type& direction,
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
    }

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

    /**
     * @brief returns a const reference to the underlying OcTree
     * @return const OcTree&
     */
    auto octree() const -> const OcTree& { return octree_; }

   private:
    OcTree octree_;

    OcTreeWrapper(const octomap_msgs::Octomap::Ptr& msg) : octree_{msg->resolution} {
        const auto& data = msg->data;
        // auto data_stream = std::istringstream{};
        // data_stream.read(char_type *s, streamsize n)
        // data_stream.read(reinterpret_cast<const char*>(&data[0]), data.size());
        // auto data_stream = std::basic_stringstream<signed char>{};
        auto data_stream = std::stringstream{};
        data_stream.write((const char*)&data[0], data.size());
        octree_.readBinaryData(data_stream);
    }

    auto raycast_(const point_type& origin, const point_type& direction, double max_range = -1,
                  bool ignore_unknown_voxels = false) const -> std::optional<point_type> {
        auto end = point_type{0, 0, 0};
        const auto intersected_occupied_voxel =
            octree_.castRay(origin, direction, end, ignore_unknown_voxels, max_range);
        if (intersected_occupied_voxel) {
            return end;
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

};  // OcTreeWrapper

void print_query_info(point3d query, OcTreeNode* node) {
    if (node != nullptr) {
        cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    } else
        cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

// auto octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg) -> void {
//     // const auto binary = msg->binary;
//     // const auto resolution = msg->resolution;
//     // const auto& data = msg->data;
// }

auto main(int argc, char* argv[]) -> int {
    ros::init(argc, argv, "octomap_test");
    auto nh = ros::NodeHandle();

    // auto octomap_sub = [&] {
    //     const auto topic_name = "/octomap_binary";
    //     const auto queue_size = 10;
    //     return nh.subscribe<octomap_msgs::Octomap>(topic_name, queue_size, octomap_cb);
    // }();

    auto clear_octomap_client = [&] {
        const auto service_name = "reset";
        return nh.serviceClient<std_srvs::Empty>(service_name);
    }();

    // /**
    //  * @brief reset octomap
    //  *
    //  */
    // const auto clear_octomap = [&] {
    //     using Request = std_srvs::Empty::Request;
    //     using Response = std_srvs::Empty::Response;
    //     auto request = Request{};
    //     auto response = Response{};

    //     clear_octomap_client.call(request, response);
    // };

    auto clear_region_of_octomap_client = [&] {
        const auto service_name = "clear_bbx";
        return nh.serviceClient<octomap_msgs::BoundingBoxQuery>(service_name);
    }();

    // /**
    //  * @brief max and min defines a bbx. All voxels in the bounding box are set to "free"
    //  */
    // const auto clear_region_of_octomap = [&](const point3d& max, const point3d& min) {
    //     using Request = octomap_msgs::BoundingBoxQuery::Request;
    //     using Response = octomap_msgs::BoundingBoxQuery::Response;

    //     const auto convert = [](const point3d& pt) {
    //         auto geo_pt = geometry_msgs::Point{};
    //         geo_pt.x = pt.x();
    //         geo_pt.y = pt.y();
    //         geo_pt.z = pt.z();
    //         return geo_pt;
    //     };
    //     auto request = Request{};
    //     request.max = convert(max);
    //     request.min = convert(min);
    //     auto response = Response{};  // is empty
    //     clear_octomap_client.call(request, response);
    // };

    const auto resolution = 0.1f;
    auto tree = octomap::OcTree(resolution);

    const auto width = 20;
    const auto depth = 1;
    const auto height = 25;
    // insert some measurements of occupied cells

    for (float x = 5; x < width + 5; x += resolution) {
        for (float y = 17; y < 17 + depth; y += resolution) {
            for (float z = 0; z < height; z += resolution) {
                // octomap::point3d endpoint((float)x * resolution * 1, (float)y * resolution * 1,
                //   (float)z * resolution * 1);
                tree.updateNode(x, y, z, true);
                // tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
            }
        }
    }

    // insert some measurements of free cells

    // for (int x = -30; x < 30; x++) {
    //     for (int y = -30; y < 30; y++) {
    //         for (int z = -30; z < 30; z++) {
    //             octomap::point3d endpoint((float)x * 0.02f - 1.0f, (float)y * 0.02f - 1.0f,
    //             (float)z * 0.02f - 1.0f); tree.updateNode(endpoint, false);  // integrate 'free'
    //             measurement
    //         }
    //     }
    // }
    cout << endl;
    cout << "performing some queries:" << endl;

    point3d query(0., 0., 0.);
    OcTreeNode* result = tree.search(query);
    print_query_info(query, result);

    query = point3d(-1., -1., -1.);
    result = tree.search(query);
    print_query_info(query, result);

    query = point3d(1., 1., 1.);
    result = tree.search(query);
    print_query_info(query, result);

    auto octree = OcTreeWrapper(std::move(tree));
    // octree.get_voxel_status_at_point(const point_type& point)

    {
        // should hit
        const auto origin = point3d(-0.5, 0.5, 0.5);
        const auto direction = point3d(1, 0, 0);
        if (const auto opt = octree.raycast(origin, direction, 1, true)) {
            const auto hit = opt.value();
            cout << "hit " << hit << endl;
        }
    }

    {
        // should NOT hit
        const auto origin = point3d(-0.5, 0.5, 10.5);
        const auto direction = point3d(1, 0, 0);
        auto end = point3d();

        const auto hit = tree.castRay(origin, direction, end, true, 1);
        if (hit) {
            cout << "hit " << end << endl;
        } else {
            cout << "no hit " << endl;
        }
    }

    {
        // should NOT hit

        const auto origin = point3d(-0.5, 0.5, 0.5);
        const auto direction = point3d(1, 0, 0);
        auto end = point3d();
        const auto hit = tree.castRay(origin, direction, end, true, 0.4);
        if (hit) {
            cout << "hit " << end << endl;
        } else {
            cout << "no hit " << endl;
        }
    }

    cout << endl;
    // tree.getRayIntersection(const point3d& origin, const point3d& direction, const point3d&
    // center, point3d& intersection)
    tree.writeBinary("data/simple_tree.bt");
    cout << "wrote example file simple_tree.bt" << endl << endl;
    cout << "now you can use octovis to visualize: octovis simple_tree.bt" << endl;
    cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl << endl;
    ros::spin();

    return 0;
}
