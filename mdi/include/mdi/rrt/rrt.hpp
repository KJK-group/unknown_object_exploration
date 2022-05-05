#pragma once

#include <ros/ros.h>

#include <cstddef>
#include <memory>

#include "mdi/common_headers.hpp"
#include "mdi/octomap.hpp"
#include "mdi/utils/random.hpp"

#ifdef USE_KDTREE
#include "kdtree/kdtree3.hpp"
#endif  // USE_KDTREE

namespace mdi::rrt {

using vec3 = Eigen::Vector3f;

class RRTBuilder;  // forward declare builder class

class RRT {
   public:
    // factory methods
    friend class RRTBuilder;
    static auto from_builder() -> RRTBuilder;
    static auto from_rosparam(std::string_view prefix) -> RRT;

    // pretty printer
    friend std::ostream& operator<<(std::ostream& os, const RRT& rrt);

    using coordinate_type = vec3;
    using waypoint_type = coordinate_type;
    using waypoints_type = std::vector<waypoint_type>;

    // TODO:
    enum class traversal_order {
        breath_first,
        depth_first,
    };

    /**
     * @brief grow the tree until a path is found, or max number of iterations is exceeded.
     *
     * @return std::optional<waypoints_type> some variant containing the waypoints if a path is
     * found. Otherwise std::nullopt.
     */
    auto run() -> std::optional<waypoints_type>;
    /**
     * @brief // grow the tree n nodes.
     * @throws std::invalid_argument if n < 0.
     * @param n
     * @return true if a path is found, false otherwise
     */
    auto growN(int n) -> bool;
    /**
     * @brief // grow the tree 1 node.
     * @return true if a path is found, false otherwise
     */
    auto grow1() -> bool { return growN(1); }

    /**
     * @brief // deallocate all nodes in the tree.
     * and reset the tree to its initial state as if constructed again with
     * the same arguments.
     */
    auto clear() -> void {
        nodes_.clear();
#ifdef USE_KDTREE
        std::for_each(kdtree3s_.begin(), kdtree3s_.end(), [&](auto& bucket) { bucket.delete_trees(); });
        kdtree3s_.erase(kdtree3s_.begin() + 1, kdtree3s_.end());
#endif  // USE_KDTREE

        remaining_iterations_ = max_iterations_;
        call_cbs_for_event_on_clearing_nodes_in_tree_();
        unregister_cbs_for_all_events();
    }

    /**
     * @brief Get the waypoints.
     * meant to be called after @ref growN or @ref grow1
     * @return std::optional<waypoint_type> some variant if a path goal has been found,
     * else std::nullopt.
     */
    auto get_waypoints() -> std::optional<waypoints_type> {
        if (waypoints_.empty()) {
            return std::nullopt;
        }
        return waypoints_;
    }
    [[nodiscard]] auto get_frontier_nodes() const -> std::vector<vec3>;

    /**
     * @brief traverse tree in breath first order, and call @ref f, for every node
     * @param f the function to call.
     */
    auto bft(const std::function<void(const vec3& pt)>& f) const -> void {
        const auto skip_root = false;
        bft_([&](const vec3& _, const vec3& pt) { f(pt); }, skip_root);
    }
    /**
     * @brief traverse tree in breath first order, and call @ref f, for every edge
     * i.e. every pair of parent and child node.
     * @param f the function to call.
     * @param skip_root if true then @ref f will be called with itself as parent and child.
     */
    auto bft(const std::function<void(const vec3& pt, const vec3& parent_pt)>& f, bool skip_root = true) const -> void {
        bft_(f, skip_root);
    }

    /**
     * @brief returns the number of nodes that are reachable from the root node, plus the root
     * itself.
     * @return std::size_t
     */
    [[nodiscard]] auto reachable_nodes() const -> std::size_t {
        std::size_t count = 0;
        bft([&](const auto& _) { ++count; });
        return count;
    }
    /**
     * @brief true, when every node is reachable from the root i.e. the tree is connected.
     */
    [[nodiscard]] auto connected() const -> bool { return reachable_nodes() == size(); }

#ifdef MEASURE_PERF
    ~RRT();

    auto enable_perf_logging(const std::filesystem::path& p) -> void {
        log_perf_measurements_enabled_ = true;
        file_path_csv_ = p;
    }
    auto disable_perf_logging() -> void { log_perf_measurements_enabled_ = false; }
#endif  // MEASURE_PERF

    auto register_cb_for_event_on_new_node_created(std::function<void(const vec3&, const vec3&)> cb) -> void {
        on_new_node_created_cb_list.push_back(cb);
    }

    auto register_cb_for_event_before_optimizing_waypoints(std::function<void(const vec3&, const vec3&)> cb) -> void {
        before_optimizing_waypoints_cb_list.push_back(cb);
    }

    auto register_cb_for_event_after_optimizing_waypoints(std::function<void(const vec3&, const vec3&)> cb) -> void {
        after_optimizing_waypoints_cb_list.push_back(cb);
    }

    auto register_cb_for_event_on_goal_reached(std::function<void(const vec3&, size_t)> cb) -> void {
        on_goal_reached_cb_list.push_back(cb);
    }
    auto register_cb_for_event_on_trying_full_path(std::function<void(const vec3&, const vec3&)> cb) -> void {
        on_trying_full_path_cb_list.push_back(cb);
    }
    auto register_cb_for_event_on_clearing_nodes_in_tree(std::function<void()> cb) -> void {
        on_clearing_nodes_in_tree_cb_list.push_back(cb);
    }

    auto register_cb_for_event_on_raycast(std::function<void(const vec3&, const vec3&, const float, bool)> cb) -> void {
        on_raycast_cb_list.push_back(cb);
    }

    auto unregister_cbs_for_event_on_new_node_created() -> void { on_new_node_created_cb_list.clear(); }
    auto unregister_cbs_for_event_on_trying_full_path() -> void { on_trying_full_path_cb_list.clear(); }
    auto unregister_cbs_for_event_before_optimizing_waypoints() -> void { before_optimizing_waypoints_cb_list.clear(); }
    auto unregister_cbs_for_event_after_optimizing_waypoints() -> void { after_optimizing_waypoints_cb_list.clear(); }
    auto unregister_cbs_for_event_on_goal_reached() -> void { on_goal_reached_cb_list.clear(); }
    auto unregister_cbs_for_event_on_clearing_nodes_in_tree() -> void { on_clearing_nodes_in_tree_cb_list.clear(); }
    auto unregister_cbs_for_event_on_raycast() -> void { on_raycast_cb_list.clear(); }

    auto unregister_cbs_for_all_events() -> void {
        unregister_cbs_for_event_on_new_node_created();
        unregister_cbs_for_event_on_goal_reached();
        unregister_cbs_for_event_on_trying_full_path();
        unregister_cbs_for_event_on_clearing_nodes_in_tree();
        unregister_cbs_for_event_after_optimizing_waypoints();
        unregister_cbs_for_event_before_optimizing_waypoints();
        unregister_cbs_for_event_on_raycast();
    }

    auto enable_cbs_for_event_on_new_node_created() -> void { on_new_node_created_status_ = true; }
    auto enable_cbs_for_event_before_optimizing_waypoints() -> void { before_optimizing_waypoints_status_ = true; }
    auto enable_cbs_for_event_after_optimizing_waypoints() -> void { after_optimizing_waypoints_status_ = true; }
    auto enable_cbs_for_event_on_goal_reached() -> void { on_goal_reached_status_ = true; }
    auto enable_cbs_for_event_on_trying_full_path() -> void { on_trying_full_path_status_ = true; }
    auto enable_cbs_for_event_on_clearing_nodes_in_tree() -> void { on_clearing_nodes_in_tree_status_ = true; }
    auto enable_cbs_for_event_on_raycast() -> void { on_raycast_status_ = true; }

    auto disable_cbs_for_event_on_new_node_created() -> void { on_new_node_created_status_ = false; }
    auto disable_cbs_for_event_before_optimizing_waypoints() -> void { before_optimizing_waypoints_status_ = false; }
    auto disable_cbs_for_event_after_optimizing_waypoints() -> void { after_optimizing_waypoints_status_ = false; }
    auto disable_cbs_for_event_on_goal_reached() -> void { on_goal_reached_status_ = false; }
    auto disable_cbs_for_event_on_trying_full_path() -> void { on_trying_full_path_status_ = false; }
    auto disable_cbs_for_event_on_clearing_nodes_in_tree() -> void { on_clearing_nodes_in_tree_status_ = false; }
    auto disable_cbs_for_event_on_raycast() -> void { on_raycast_status_ = false; }

    auto toggle_cbs_for_event_on_new_node_created() -> void {
        on_new_node_created_status_ = ! on_new_node_created_status_;
    }
    auto toggle_cbs_for_event_before_optimizing_waypoints() -> void {
        before_optimizing_waypoints_status_ = ! before_optimizing_waypoints_status_;
    }
    auto toggle_cbs_for_event_after_optimizing_waypoints() -> void {
        after_optimizing_waypoints_status_ = ! after_optimizing_waypoints_status_;
    }
    auto toggle_cbs_for_event_on_goal_reached() -> void { on_goal_reached_status_ = ! on_goal_reached_status_; }
    auto toggle_cbs_for_event_on_trying_full_path() -> void {
        on_trying_full_path_status_ = ! on_trying_full_path_status_;
    }
    auto toggle_cbs_for_event_on_clearing_nodes_in_tree() -> void {
        on_clearing_nodes_in_tree_status_ = ! on_clearing_nodes_in_tree_status_;
    }

    auto toggle_cbs_for_event_on_raycast() -> void { on_raycast_status_ = ! on_clearing_nodes_in_tree_status_; }

    [[nodiscard]] auto empty() const -> bool { return nodes_.empty(); }
    [[nodiscard]] auto size() const -> std::size_t { return nodes_.size(); };
    [[nodiscard]] auto remaining_iterations() const -> int { return remaining_iterations_; }
    [[nodiscard]] auto max_iterations() const -> std::size_t { return max_iterations_; }
    [[nodiscard]] auto sampling_radius() const -> float { return sampling_radius_; }
    [[nodiscard]] auto start_position() const -> vec3 { return start_position_; }
    [[nodiscard]] auto goal_position() const -> vec3 { return goal_position_; }

    auto assign_octomap(mdi::Octomap* map) -> void { octomap_ = map; };

   private:
    RRT() = default;
    RRT(const vec3& start_position, const vec3& goal_position, float step_size, float goal_bias,
        std::size_t max_iterations, float max_dist_goal_tolerance,
        float probability_of_testing_full_path_from_new_node_to_goal);

    struct node_t {
       public:
        node_t(vec3 pos, node_t* parent_ = nullptr) : parent(parent_), children{}, position_(std::move(pos)) {}

        node_t* parent = nullptr;
        std::vector<node_t*> children{};
        vec3 position_{};

        [[nodiscard]] auto is_leaf() const -> bool { return children.empty(); }
        [[nodiscard]] auto is_root() const -> bool { return parent == nullptr; }
        [[nodiscard]] auto number_of_children() const -> std::size_t { return children.size(); }

        friend std::ostream& operator<<(std::ostream& ostream, const node_t& n) {
            ostream << "RRT::node:\n";
            ostream << "  root: " << (n.is_root() ? "true" : "false") << '\n';
            ostream << "  leaf: " << (n.is_leaf() ? "true" : "false") << '\n';
            ostream << "  position:\n";
            ostream << "    x: " << n.position_.x() << '\n';
            ostream << "    y: " << n.position_.y() << '\n';
            ostream << "    z: " << n.position_.z() << '\n';
            ostream << "  number_of_children: " << n.number_of_children() << '\n';
            return ostream;
        }
    };

    auto sample_random_point_() -> vec3;
    // TODO: make kdtree or list transparent to the caller
    auto find_nearest_neighbor_(const vec3& pt) -> node_t*;

    /**
     * @brief traverse tree in breath first order, and call @ref f, for every edge
     * i.e. every pair of parent and child node.
     * @param f the function to call.
     * @param skip_root if true then @ref f will be called with itself as parent and child.
     */
    auto bft_(const std::function<void(const vec3& parent_pt, const vec3& child_pt)>& f, bool skip_root = true) const
        -> void;
    [[nodiscard]] auto grow_() -> bool;

    auto insert_node_(const vec3& pos, node_t* parent) -> node_t&;

    auto backtrack_and_set_waypoints_starting_at_(node_t* start_node) -> bool;
    auto optimize_waypoints_() -> void;

    auto collision_free_(const vec3& a, const vec3& b, float x, float y, float z, float padding,
                         float end_of_raycast_padding = 1.0f) const -> bool;
    inline auto collision_free_(const vec3& a, const vec3& b, float r, float padding,
                                float end_of_raycast_padding = 1.0f) const -> bool {
        return collision_free_(a, b, r, r, r, padding, end_of_raycast_padding);
    }
    inline auto collision_free_(const vec3& a, const vec3& b) const -> bool {
        return collision_free_(a, b, drone_radius_, padding_, end_of_raycast_padding_);
    }

    float drone_radius_ = 2.5f;
    float padding_ = 0.1f;
    float end_of_raycast_padding_ = 1.0f;

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
    int remaining_iterations_{};
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
    std::vector<vec3> waypoints_{};
    std::vector<node_t> nodes_{};

    std::size_t linear_search_start_index_{0};

    mdi::Octomap* octomap_ = nullptr;

#ifdef USE_KDTREE

    static constexpr std::int32_t max_number_of_kdtrees_per_bucket_ = 10;
    static constexpr std::int32_t max_number_of_nodes_to_do_linear_search_on_ = 1000;
    std::int32_t n_kdtree_nodes_ = 0;

    using kdtree3 = kdtree::kdtree3<std::size_t>;

    struct kdtree3_bucket_t {
        int size_of_a_tree;
        // TODO: use std::unique_ptr
        std::array<kdtree3*, max_number_of_kdtrees_per_bucket_> forest{};
        kdtree3_bucket_t(int size_of_a_tree_) : size_of_a_tree(size_of_a_tree_) {}

        [[nodiscard]] auto number_of_trees() const {
            return std::count_if(forest.begin(), forest.end(), [](const auto& tree) { return tree != nullptr; });
        }
        [[nodiscard]] auto bucket_is_full() const -> bool {
            return number_of_trees() == max_number_of_kdtrees_per_bucket_;
        }
        [[nodiscard]] auto empty() const { return number_of_trees() == 0; }

        [[nodiscard]] auto number_of_nodes_in_bucket() const { return number_of_trees() * size_of_a_tree; }

        auto delete_trees() {
            std::for_each(forest.begin(), forest.end(), [&](auto& tree) {
                if (tree != nullptr) {
                    delete tree;
                    tree = nullptr;
                }
            });
        }

        auto add_kdtree(kdtree3::fn&& fn, int size) {
            if (bucket_is_full()) {
                throw std::runtime_error("bucket is full");
            }

            if (size != size_of_a_tree) {
                // should never happen
                std::exit(EXIT_FAILURE);
            }

            if (size < 0) {
                throw std::runtime_error("size cannot be negative");
            }

            auto index = number_of_trees();
            index -= index > 0 ? -1 : 0;  // handle index
            forest[index] = new kdtree3(std::move(fn), size);
        }

    };  // kdtree3_forest_t

    // precompute
    std::vector<kdtree3_bucket_t> kdtree3s_{{max_number_of_nodes_to_do_linear_search_on_}};

    auto add_bucket() -> void {
        kdtree3s_.emplace_back(static_cast<int>(max_number_of_nodes_to_do_linear_search_on_ *
                                                std::pow(max_number_of_kdtrees_per_bucket_, kdtree3s_.size())));
    }

#endif  // USE_KDTREE

    mdi::utils::random::random_point_generator rng_{0.0, 1.0};

    template <typename... Ts>
    using action = std::function<void(Ts...)>;

    auto call_cbs_for_event_on_new_node_created_(const vec3&, const vec3&) const -> void;
    auto call_cbs_for_event_on_goal_reached_(const vec3&) const -> void;
    auto call_cbs_for_event_on_trying_full_path_(const vec3&, const vec3&) const -> void;
    auto call_cbs_for_event_on_clearing_nodes_in_tree_() const -> void;
    auto call_cbs_for_event_after_optimizing_waypoints_(const vec3&, const vec3&) const -> void;
    auto call_cbs_for_event_before_optimizing_waypoints_(const vec3&, const vec3&) const -> void;
    auto call_cbs_for_event_on_raycast_(const vec3&, const vec3&, const float, bool) const -> void;

    std::vector<std::function<void(const vec3&, const vec3&)>> on_new_node_created_cb_list{};
    std::vector<std::function<void(const vec3&, size_t)>> on_goal_reached_cb_list{};
    std::vector<std::function<void(const vec3&, const vec3&)>> on_trying_full_path_cb_list{};
    std::vector<std::function<void()>> on_clearing_nodes_in_tree_cb_list{};
    std::vector<std::function<void(const vec3&, const vec3&)>> before_optimizing_waypoints_cb_list{};
    std::vector<std::function<void(const vec3&, const vec3&)>> after_optimizing_waypoints_cb_list{};
    std::vector<std::function<void(const vec3&, const vec3&, const float, bool)>> on_raycast_cb_list{};

    bool on_new_node_created_status_ = true;
    bool on_goal_reached_status_ = true;
    bool on_trying_full_path_status_ = true;
    bool on_clearing_nodes_in_tree_status_ = true;
    bool before_optimizing_waypoints_status_ = true;
    bool after_optimizing_waypoints_status_ = true;
    bool on_raycast_status_ = false;

#ifdef MEASURE_PERF
    bool log_perf_measurements_enabled_ = false;
    std::filesystem::path file_path_csv_;
    using microseconds_t = std::chrono::duration<double, std::nano>;
    std::vector<std::chrono::nanoseconds> timimg_measurements_{};
#endif  // MEASURE_PERF
};

}  // namespace mdi::rrt
