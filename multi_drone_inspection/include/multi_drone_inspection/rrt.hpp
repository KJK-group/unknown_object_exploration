#include <eigen3/Eigen/Dense>
#include <functional>
#include <optional>
#include <vector>

// #include "multi_drone_inspection/voxblox_manager.hpp"
#include "multi_drone_inspection/voxelgrid.hpp"

namespace mpi {
using std::vector, std::optional;

// using Vector3f = Eigen::Vector3f;
// using Waypoints = std::vector<Vector3f>;

/**
 * @brief
 * @note this RRT implementation only considers points in 3D space.
 */
class RRT {
   public:
    RRT(Vector3f start_pos_, Vector3f goal_pos_, float step_size_, unsigned int max_iterations_,
        float max_dist_goal_tolerance_, VoxelGrid voxelgrid_);

    ~RRT();
    /**
     * @brief
     *
     * @return optional<vector<Vector3f>>
     */
    auto run() -> optional<vector<Vector3f>>;
    /**
     * @brief grow the tree n nodes.
     *
     */
    auto growN(unsigned int n) -> void;

    /**
     * @brief grow the tree one node.
     *
     * @return decltype(growN(1))
     */
    auto grow1() -> decltype(growN(1)) { return growN(1); }
    /**
     * @brief deallocate all nodes in the tree.
     *
     * @return true
     * @return false
     */
    auto clear() -> bool;
    auto get_root_node() -> const Node&;
    /**
     * @brief return the number of nodes in the tree.
     *
     * @return unsigned int
     */
    auto size() -> unsigned int { return n_nodes; };
    auto get_waypoints() -> vector<Vector3f>;

   private:
    struct Node {
        Node(const Vector3f& position_, Node* parent_ = nullptr)
            : position(position_), parent(parent_) {
            if (parent != nullptr) {
                parent->children.push_back(this);
            }
        }

        ~Node() {}

        auto get_parent() const { return parent; }

        auto depth() const -> unsigned int {
            auto n = 0;
            for (const auto ancestor = parent; ancestor != nullptr; ancestor = ancestor->parent) {
                ++n;
            }
            return n;
        }

       private:
        Vector3f position;
        Node* parent = nullptr;
        vector<Node> children;
    };

    float step_size_;
    float max_step_size_;
    bool adaptive_step_size_control_enabled_ = false;
    float goal_bias_ = 0.0f;
    float waypoint_bias_ = 0.0f;
    /**
     * The maximum number of random states in the state-space that we will
     * try before giving up on finding a path to the goal.
     */
    unsigned int max_iterations_;
    float max_dist_goal_tolerance_;
    Vector3f start_position_;
    Vector3f goal_position_;

    float sampling_radius_;
    /**
     * @brief the interface which is used to query the voxel grid about
     * occupancy status.
     */

    VoxelGrid voxelgrid_;

    unsigned int n_nodes_ = 0;  // number of nodes in the tree
};

}  // namespace mpi
