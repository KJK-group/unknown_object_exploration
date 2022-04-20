#ifndef _MULTI_DRONE_INSPECTION_RRT_HPP_
#define _MULTI_DRONE_INSPECTION_RRT_HPP_

#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <forward_list>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ratio>
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
    static auto from_builder() -> RRTBuilder;
    static auto from_rosparam(std::string_view prefix) -> RRT;

#ifdef MEASURE_PERF
    ~RRT() {
        if (log_perf_measurements_enabled_) {
            // std::filesystem::is_regular_file(file_path_csv_);
            // const auto path = std::filesystem::current_path() / "perf.csv";
            std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
                      << " path = " << file_path_csv_ << '\n';

            auto file = std::ofstream(file_path_csv_);
            if (file.is_open()) {
                file << "t,nodes\n";
                const auto nanoseconds_to_seconds = [](double nanoseconds) {
                    return nanoseconds / 1e9;
                };

                for (std::size_t i = 0; i < timimg_measurements_.size(); ++i) {
                    // std::nth_element(timimg_measurements_.cb)
                    const auto& measurement = timimg_measurements_[i];
                    file << i << ","
                         << nanoseconds_to_seconds(static_cast<double>(measurement.count()))
                         << '\n';
                }
                file.close();
            }
        }
    }

    auto enable_perf_logging(const std::filesystem::path& p) -> void {
        log_perf_measurements_enabled_ = true;
        file_path_csv_ = p;
    }
    auto disable_perf_logging() -> void { log_perf_measurements_enabled_ = false; }
#endif  // MEASURE_PERF

    // FIXME: remove this
    auto print_each_node() const -> void {
        for (const auto& n : nodes_) {
            std::cout << n << '\n';
        }
    }
    // FIXME: remove this
    auto print_number_of_root_nodes() -> void {
        const auto is_root = [](const node_t& n) { return n.is_root(); };
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
        call_cbs_for_event_on_clearing_nodes_in_tree_();
    }

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
    [[nodiscard]] auto empty() const -> bool { return nodes_.empty(); }
    [[nodiscard]] auto size() const -> std::size_t { return nodes_.size(); };
    [[nodiscard]] auto remaining_iterations() const -> int { return remaining_iterations_; }
    [[nodiscard]] auto max_iterations() const -> std::size_t { return max_iterations_; }
    [[nodiscard]] auto sampling_radius() const -> float { return sampling_radius_; }
    [[nodiscard]] auto start_position() const -> vec3 { return start_position_; }
    [[nodiscard]] auto goal_position() const -> vec3 { return goal_position_; }

   private:
    RRT() = default;
    RRT(const vec3& start_position, const vec3& goal_position, float step_size, float goal_bias,
        std::size_t max_iterations, float max_dist_goal_tolerance,
        float probability_of_testing_full_path_from_new_node_to_goal);

    struct node_t {
       public:
        node_t(vec3 pos, node_t* parent_ = nullptr)
            : parent(parent_), children{}, position_(std::move(pos)) {}

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
    auto find_nearest_neighbor_(const vec3& point) -> node_t*;
    auto bft_(const std::function<void(const vec3& pt, const vec3& parent_pt)>& f) const -> void;
    [[nodiscard]] auto grow_() -> bool;
    auto insert_node_(const vec3& pos, node_t* parent) -> node_t&;

    auto backtrack_and_set_waypoints_(node_t* start_node) -> bool;

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
    std::int32_t kdtree_size_ = 1000;
    static constexpr std::int32_t max_number_of_kdtrees_{4};
    static constexpr std::int32_t max_number_of_nodes_to_do_linear_search_on_ = 1000;
    using kdtree3 = kdtree::kdtree<float, std::size_t, 3>;
    std::vector<kdtree3*> kdtree3s_;
    // std::vector<std::unique_ptr<kdtree3>> kdtree3s_;
    // std::forward_list<std::unique_ptr<kdtree3>> kdtree3s_;

    mdi::utils::random::random_point_generator rng_{0.0, 1.0};

    template <typename... Ts>
    using action = std::function<void(Ts...)>;
    // template <typename... Ts>
    // using callbacks = std::vector<action>;

    auto call_cbs_for_event_on_new_node_created_(const vec3&, const vec3&) const -> void;
    auto call_cbs_for_event_on_goal_reached_(const vec3&) const -> void;
    auto call_cbs_for_event_on_trying_full_path_(const vec3&, const vec3&) const -> void;
    auto call_cbs_for_event_on_clearing_nodes_in_tree_() const -> void;

    std::vector<std::function<void(const vec3&, const vec3&)>> on_new_node_created_cb_list{};
    std::vector<std::function<void(const vec3&, size_t)>> on_goal_reached_cb_list{};
    std::vector<std::function<void(const vec3&, const vec3&)>> on_trying_full_path_cb_list{};
    std::vector<std::function<void()>> on_clearing_nodes_in_tree_cb_list{};

#ifdef MEASURE_PERF
    bool log_perf_measurements_enabled_ = false;
    std::filesystem::path file_path_csv_;
    using microseconds_t = std::chrono::duration<double, std::nano>;
    std::vector<std::chrono::nanoseconds> timimg_measurements_{};
#endif  // MEASURE_PERF

};  // class RRT

}  // namespace mdi::rrt

#endif  // _MULTI_DRONE_INSPECTION_RRT_HPP_
