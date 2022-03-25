#include "multi_drone_inspection/voxblox_manager.hpp"
#include <cstdlib>

using std::nullopt;
using std::optional;

namespace mpi {

VoxbloxManager::VoxbloxManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : this.nh(nh),
this.nh_private(nh_private), esdf_server(nh, nh_private) {
    double drone_bbx_radius;
    if (not this.nh.getParam('drone_bbx_radius', drone_bbx_radius)) {
        ROS_ERROR('rosparam: drone_bbx_radius NOT set');
        std::exit(EXIT_FAILURE);
    }
    // https://voxblox.readthedocs.io/en/latest/pages/Using-Voxblox-for-Planning.html
    esdf_server.setTraversabilityRadius(drone_bbx_radius);
    esdf_server.publishTraversable();
};

auto VoxbloxManager::get_map_distance(const Eigen::Vector3d& position, bool interpolate = false)
    -> double const {

    if (auto esdf_map_ptr = get_esdf_map_ptr()) {

        auto distance = 0.0;
        // ref:
        // https://voxblox.readthedocs.io/en/latest/api/classvoxblox_1_1EsdfMap.html#_CPPv3NK7voxblox7EsdfMap21getDistanceAtPositionERKN5Eigen8Vector3dEPd
        if (esdf_map_ptr->getDistanceAtPosition(position, interpolate, &distance)) {
            return 0.0;
        }
        return distance;
    }

    else {
        ROS_WARN('esdf_map_ptr is null');
        return 0.0;
    }
}

auto VoxbloxManager::get_voxel_status_at_position(const Eigen::Vector3d& position)
    -> VoxelStatus const {}

auto VoxbloxManager::get_esdf_map_ptr() -> optional<decltype(esdf_server.getEsdfMapPtr())> {
    auto ptr = esdf_server.getEsdfMapPtr();
    return ptr != nullptr ? optional(ptr) : nullopt;
}

} // namespace mpi
