#ifndef _MULTI_DRONE_INSPECTION_RRT_HPP_
#define _MULTI_DRONE_INSPECTION_RRT_HPP_

#include <eigen3/Eigen/Dense>
#include <functional>
#include <optional>
#include <vector>

#include "kdtree/kdtree.hpp"

namespace mdi::rrt {

using vec3 = Eigen::Vector3f;

class RRTBuilder;  // forward declare builder class

class RRT {
   public:
    friend class RRTBuilder;
    // friend class builder;
    static RRTBuilder builder();

    auto run() -> std::optional<std::vector<vec3>>;
    // auto growN(std::size_t n) -> void;                       // grow the tree n nodes.
    // auto grow1() -> decltype(growN(1)) { return growN(1); }  // grow the tree one node.
    // auto clear() -> bool;                                    // deallocate all nodes in the tree.
    // auto get_root_node() const -> vec3;
    auto size() const -> std::size_t { return nodes_.size(); };
    // auto get_waypoints() -> std::vector<vec3>;
    auto unregister_cbs_for_event_on_new_node_created() -> void {
        on_new_node_created_cb_list.clear();
    }
    auto unregister_cbs_for_event_on_goal_reached() -> void { on_goal_reached_cb_list.clear(); }

   private:
    RRT() = default;

    struct node {
        node(const vec3& pos, node* parent_ = nullptr)
            : position(pos), parent(parent_), children{} {}
        vec3 position;
        node* parent;
        std::vector<node*> children;
    };

    auto sample_random_point() -> vec3;
    auto get_nearest_neighbor(const vec3& point) -> node&;

    // RRT(vec3 start_pos_, vec3 goal_pos_, float step_size_, unsigned int max_iterations_,
    // float max_dist_goal_tolerance_, VoxelGrid voxelgrid_);

    // struct Node {
    //     Node(const vec3& position_, Node* parent_ = nullptr)
    //         : position(position_), parent(parent_) {
    //         if (parent != nullptr) {
    //             parent->children.push_back(this);
    //         }
    //     }

    //     auto get_parent() const { return parent; }

    //     auto depth() const -> unsigned int {
    //         auto n = 0;
    //         for (const auto ancestor = parent; ancestor != nullptr; ancestor = ancestor->parent)
    //         {
    //             ++n;
    //         }
    //         return n;
    //     }

    //    private:
    //     vec3 position;
    //     Node* parent = nullptr;
    //     std::vector<Node> children;
    // };

    float step_size_;
    // float max_step_size_;
    // bool adaptive_step_size_control_enabled_ = false;
    float goal_bias_ = 0.0f;
    // float waypoint_bias_ = 0.0f;
    /**
     * The maximum number of random states in the state-space that we will
     * try before giving up on finding a path to the goal.
     */
    std::size_t max_iterations_;
    float max_dist_goal_tolerance_;
    vec3 start_position_;
    vec3 goal_position_;
    float sampling_radius_;
    /**
     * @brief the interface which is used to query the voxel grid about
     * occupancy status.
     */
    // TODO: implement
    // VoxelGrid voxelgrid_;
    using kdtree3 = kdtree::kdtree<float, 3>;
    kdtree3* kdtree3_;

    std::vector<std::function<void(const vec3&, const vec3&)>> on_new_node_created_cb_list;
    std::vector<std::function<void(const vec3&, size_t)>> on_goal_reached_cb_list;

    std::vector<node> nodes_;
    std::size_t n_nodes_ = 0;  // number of nodes in the tree
};



}  // namespace mdi::rrt

#endif  // _MULTI_DRONE_INSPECTION_RRT_HPP_
