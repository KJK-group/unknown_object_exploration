#ifndef _MULTI_DRONE_INSPECTION_KDTREE3_HPP_
#define _MULTI_DRONE_INSPECTION_KDTREE3_HPP_

#include <eigen3/Eigen/Dense>
#include <optional>
#include <utility>
#include <vector>

namespace kdtree {

template <typename value_type>
class kdtree3 final {
   public:
    using vec3 = Eigen::Vector3f;
    using distance_type = double;
    using point_type = vec3;
    using coordinate_type = float;

    kdtree3(const kdtree3&) = delete;             // remove copy constructor
    kdtree3& operator=(const kdtree3&) = delete;  // remove copy assignment operator

    /**
     * Constructor taking a pair of iterators. Adds each
     * point in the range [begin, end) to the tree.
     *
     * @param begin start of range
     * @param end end of range
     */
    template <typename iterator>
    kdtree3(iterator begin, iterator end) : nodes_(begin, end) {
        root_ = make_tree_(0, nodes_.size(), 0);
    }

    /**
     * Constructor taking a function object that generates
     * points. The function object will be called n times
     * to populate the tree.
     *
     * @param f function that returns a point
     * @param n number of points to add
     */
    using fn = std::function<std::pair<point_type, value_type>()>;
    kdtree3(fn&& f, int n) {
        if (n < 0) {
            throw std::invalid_argument("n must be positive");
        }

        nodes_.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const auto&& [point, value] = f();
            nodes_.emplace_back(point, value);
        }
        root_ = make_tree_(0, nodes_.size(), 0);
    }

    [[nodiscard]] auto size() const { return nodes_.size(); }
    [[nodiscard]] auto empty() const { return nodes_.empty(); }

    /**
     * @return size_t the number of nodes visited by the last call to @ref
     * nearest().
     */
    [[nodiscard]] auto n_visited() const { return n_visited_; }

    /**
     * @brief distance between the input point and return value
     * from the last call to nearest().
     */
    [[nodiscard]] auto distance() const { return std::sqrt(best_distance_); }

    struct nearest_neighbor_result {
        point_type point;
        distance_type distance{};
        value_type value;
    };  // nearest_neighbor_result

    /**
     * Finds the nearest point in the tree to the given point.
     * It is not valid to call this function if the tree is empty.
     *
     * @param pt a point
     * @return the nearest point in the tree to the given point
     */
    auto nearest(const point_type& pt) -> std::optional<nearest_neighbor_result> {
        if (root_ == nullptr) {
            return std::nullopt;
        }
        best_ = nullptr;
        n_visited_ = 0;
        best_distance_ = 0;
        nearest_(root_, pt, 0);
        return nearest_neighbor_result{best_->point_, best_distance_, best_->value_};
        // return std::make_pair(best_->point_, best_->value_);
    }

    static constexpr int dimensions = 3;

   private:
    struct node_t {
        node_t(point_type pt, const value_type& value)
            : point_(std::move(pt)), value_(value), left_(nullptr), right_(nullptr) {}

        // TODO: use squared distance instead of sqrt distance. as the size is not of interest, but
        //  only relative ordering of distances between points.
        [[nodiscard]] auto distance(const point_type& pt) const -> double { return (point_ - pt).norm(); }
        [[nodiscard]] auto get_element_in_dimension(int index) const -> coordinate_type {
            assert(0 <= index && index < 3);
            return point_(index);
        }

        point_type point_;
        value_type value_;
        node_t* left_;
        node_t* right_;
    };

    struct dimension_comparator {
        dimension_comparator(int index) : index_(index) {}
        bool operator()(const node_t& node1, const node_t& node2) const {
            return node1.get_element_in_dimension(index_) < node2.get_element_in_dimension(index_);
        }
        int index_;
    };

    node_t* root_ = nullptr;
    node_t* best_ = nullptr;
    double best_distance_ = 0;
    std::size_t n_visited_ = 0;
    std::vector<node_t> nodes_{};

    auto make_tree_(std::size_t begin, std::size_t end, std::size_t index) -> node_t* {
        if (end <= begin) {
            return nullptr;
        }
        std::size_t n = begin + (end - begin) / 2;
        auto itr = nodes_.begin();
        // compute median
        std::nth_element(itr + begin, itr + n, itr + end, dimension_comparator(index));
        index = (index + 1) % dimensions;
        nodes_[n].left_ = make_tree_(begin, n, index);
        nodes_[n].right_ = make_tree_(n + 1, end, index);
        return &nodes_[n];
    }

    auto nearest_(node_t* root, const point_type& point, std::size_t index) -> void {
        if (root == nullptr) {
            return;
        }
        ++n_visited_;
        const auto distance = root->distance(point);
        // update best candidate if distance is better than the current best.
        if (best_ == nullptr || distance < best_distance_) {
            best_distance_ = distance;
            best_ = root;
        }
        // can not get a better distance
        if (best_distance_ == 0) {
            return;
        }
        // if dx is positive then point is less than root in this dimension,
        // and we should go left. otherwise right.
        const auto dx = root->get_element_in_dimension(index) - point(index);
        const auto less_than = dx > 0;
        // get next axis
        index = (index + 1) % dimensions;
        nearest_(less_than ? root->left_ : root->right_, point, index);
        // check hypersphere intersection
        if (dx * dx >= best_distance_) {
            // we can ignore the other subtree as the hypersphere does not intersect.
            return;
        }
        // we have to check the other subtree as well
        nearest_(less_than ? root->right_ : root->left_, point, index);
    }

};  // kdtree3

}  // namespace kdtree

#endif  // _MULTI_DRONE_INSPECTION_KDTREE3_HPP_
