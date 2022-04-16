#ifndef _MULTI_DRONE_INSPECTION_RRT_HPP_
#define _MULTI_DRONE_INSPECTION_RRT_HPP_

#include <ros/ros.h>

#include <algorithm>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "kdtree/kdtree.hpp"
#include "multi_drone_inspection/utils/random.hpp"

namespace mdi::rrt {

using vec3 = Eigen::Vector3f;

class RRTBuilder;  // forward declare builder class

class RRT {
   public:
    friend class RRTBuilder;
    static RRTBuilder builder();

    static auto from_rosparam(std::string_view prefix) -> RRT;

    auto print_each_node() const -> void {
        for (const auto& n : nodes_) {
            std::cout << n << '\n';
        }
    }

    auto print_number_of_root_nodes() -> void {
        const auto is_root = [](const node& n) { return n.is_root(); };
        const auto count = std::count_if(nodes_.cbegin(), nodes_.cend(), is_root);
        std::cout << "number of root nodes" << count << '\n';
    }

    friend std::ostream& operator<<(std::ostream& os, const RRT& rrt);

    auto run() -> std::optional<std::vector<vec3>>;
    auto growN(int n) -> bool;                 // grow the tree n nodes.
    auto grow1() -> bool { return growN(1); }  // grow the tree one node.
                                               /**
                                                * @brief // deallocate all nodes in the tree.
                                                */
    auto clear() -> void {
        nodes_.clear();
        call_cbs_for_event_on_clearing_nodes_in_tree();
    }

    [[nodiscard]] auto size() const -> std::size_t { return nodes_.size(); };
    auto get_waypoints() -> std::optional<std::vector<vec3>> {
        if (waypoints_.empty()) {
            return std::nullopt;
        }
        return waypoints_;
    }
    [[nodiscard]] auto get_frontier_nodes() const -> std::vector<vec3>;
    auto bft(const std::function<void(const vec3& pt)>& f) -> void;
    auto bft(const std::function<void(const vec3& pt, const vec3& parent_pt)>& f) -> void;

    auto connectivity() const -> std::size_t {
        std::size_t n = 0;
        // return std::count_if(nodes_.cbegin(), nodes_.cend(), [](const node& n) { n. ; })
        bft_([&](const auto& p1, const auto& p2) { ++n; });
        return n;
    }

    auto fully_connected() const -> bool { return connectivity() == size(); }

    auto register_cb_for_event_on_new_node_created(std::function<void(const vec3&, const vec3&)> cb)
        -> void {
        on_new_node_created_cb_list.push_back(cb);
    }
    auto register_cb_for_event_on_goal_reached(std::function<void(const vec3&, size_t)> cb)
        -> void {
        on_goal_reached_cb_list.push_back(cb);
    }
    auto register_cb_for_event_on_trying_full_path(std::function<void(const vec3&, const vec3&)> cb)
        -> void {
        on_trying_full_path_cb_list.push_back(cb);
    }
    auto register_cb_for_event_on_clearing_nodes_in_tree(std::function<void()> cb) -> void {
        on_clearing_nodes_in_tree_cb_list.push_back(cb);
    }

    auto unregister_cbs_for_event_on_new_node_created() -> void {
        on_new_node_created_cb_list.clear();
    }

    auto unregister_cbs_for_event_on_trying_full_path() -> void {
        on_trying_full_path_cb_list.clear();
    }

    auto unregister_cbs_for_event_on_goal_reached() -> void { on_goal_reached_cb_list.clear(); }
    auto unregister_cbs_for_event_on_clearing_nodes_in_tree() -> void {
        on_clearing_nodes_in_tree_cb_list.clear();
    }
    auto unregister_cbs_for_all_events() -> void {
        unregister_cbs_for_event_on_new_node_created();
        unregister_cbs_for_event_on_goal_reached();
        unregister_cbs_for_event_on_trying_full_path();
        unregister_cbs_for_event_on_clearing_nodes_in_tree();
    }

    [[nodiscard]] auto get_number_of_nodes() const noexcept -> std::size_t { return nodes_.size(); }
    [[nodiscard]] std::size_t remaining_iterations() const noexcept {
        return remaining_iterations_;
    }
    [[nodiscard]] std::size_t max_iterations() const  { return max_iterations_; }
    [[nodiscard]] float sampling_radius() const  { return sampling_radius_; }
    [[nodiscard]] auto start_position() const -> vec3 { return start_position_; }
    [[nodiscard]] auto goal_position() const -> vec3 { return goal_position_; }

   private:
    RRT() = default;
    RRT(const vec3& start_position, const vec3& goal_position, float step_size, float goal_bias,
        std::size_t max_iterations, float max_dist_goal_tolerance,
        float probability_of_testing_full_path_from_new_node_to_goal);

    struct node {
       public:
        node(vec3 pos, node* parent_ = nullptr)
            : parent(parent_), children{}, position_(std::move(pos)) {}

        node* parent = nullptr;
        std::vector<node*> children{};
        vec3 position_{};

        [[nodiscard]] auto is_leaf() const -> bool { return children.empty(); }
        [[nodiscard]] auto is_root() const -> bool { return parent == nullptr; }
        [[nodiscard]] auto get_number_of_children() const -> std::size_t { return children.size(); }
        friend std::ostream& operator<<(std::ostream& os, const node& n) {
            os << "RRT::node:\n";
            os << "  root: " << (n.is_root() ? "true" : "false") << '\n';
            os << "  leaf: " << (n.is_leaf() ? "true" : "false") << '\n';
            os << "  position:\n";
            os << "    x: " << n.position_.x() << '\n';
            os << "    y: " << n.position_.y() << '\n';
            os << "    z: " << n.position_.z() << '\n';
            os << "  number_of_children: " << n.get_number_of_children() << '\n';
            return os;
        }
    };

    using kdtree3 = kdtree::kdtree<float, 3>;
    kdtree3* kdtree3_;

    // todo: merge kdtree and list with by using the address of a leaf in kdtree,
    // as the parent of the "root" node in the linear list.
    // class backing_collection {
    //    public:
    //     backing_collection() = default;
    //     // virtual ~backing_collection() {}
    // 	insert

    //    private:
    //     std::vector<node> nodes_{};
    //     std::vector<kdtree3> kdtrees_{};
    // };

    auto sample_random_point() -> vec3;
    auto get_nearest_neighbor(const vec3& point) -> node*;
    auto bft_(const std::function<void(const vec3& pt, const vec3& parent_pt)>& f) const -> void;
    [[nodiscard]] auto grow_() -> bool;

    float step_size_{};
    // float max_step_size_;
    // bool adaptive_step_size_control_enabled_ = false;
    float goal_bias_ = 0.0f;
    /**
     * @brief should be between 0 and 1.
     */
    float probability_of_testing_full_path_from_new_node_to_goal_ = 0.0f;
    // float waypoint_bias_ = 0.0f;
    /**
     * The maximum number of random states in the state-space that we will
     * try before giving up on finding a path to the goal.
     */
    std::size_t max_iterations_{};
    std::size_t remaining_iterations_{};

    float max_dist_goal_tolerance_{};
    vec3 start_position_{};
    vec3 goal_position_{};
    vec3 direction_from_start_to_goal_{};

    float sampling_radius_{};
    /**
     * @brief the interface which is used to query the voxel grid about
     * occupancy status.
     */
    // TODO: implement
    // VoxelGrid voxelgrid_;

    mdi::utils::random::random_point_generator rng_{0.0, 1.0};

    template <typename... Ts>
    using action = std::function<void(Ts...)>;
    // template <typename... Ts>
    // using callbacks = std::vector<action>;

    auto call_cbs_for_event_on_new_node_created(const vec3&, const vec3&) const -> void;
    auto call_cbs_for_event_on_goal_reached(const vec3&) const -> void;
    auto call_cbs_for_event_on_trying_full_path() const -> void;
    auto call_cbs_for_event_on_clearing_nodes_in_tree() const -> void;

    std::vector<std::function<void(const vec3&, const vec3&)>> on_new_node_created_cb_list{};
    std::vector<std::function<void(const vec3&, size_t)>> on_goal_reached_cb_list{};
    std::vector<std::function<void(const vec3&, const vec3&)>> on_trying_full_path_cb_list{};
    std::vector<std::function<void()>> on_clearing_nodes_in_tree_cb_list{};

    std::vector<vec3> waypoints_{};
    std::vector<node> nodes_{};
};  // namespace mdi::rrt

}  // namespace mdi::rrt

#endif  // _MULTI_DRONE_INSPECTION_RRT_HPP_
