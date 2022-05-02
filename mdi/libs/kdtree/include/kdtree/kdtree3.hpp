#ifndef _MULTI_DRONE_INSPECTION_KDTREE3_HPP_
#define _MULTI_DRONE_INSPECTION_KDTREE3_HPP_

#include <array>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace kdtree {

template <typename Value>
class kdtree3 final {
   public:
    using vec3 = Eigen::Vector3f;
    using distance_type = double;
    using Point = vec3;
    using coordinate_type = float;
    struct RangeQuery {
        RangeQuery(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max)
            : min{x_min, y_min, z_min}, max{x_max, y_max, z_max} {}
        Point min, max;
    };

    struct RangeQueryResult {
        Point point;
        Value value;
    };

    struct NearestNeighborResult {
        Point point;
        distance_type distance{};
        Value value;
    };

    static constexpr int dimensions = 3;

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
    using fn = std::function<std::pair<Point, Value>()>;
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

    /**
     * Finds the nearest point in the tree to the given point.
     * It is not valid to call this function if the tree is empty.
     *
     * @param pt a point
     * @return the nearest point in the tree to the given point
     */
    [[nodiscard]] auto nearest(const Point& pt) -> std::optional<NearestNeighborResult> {
        if (root_ == nullptr) {
            // return std::nullopt;
            return {};
        }
        best_ = nullptr;
        n_visited_ = 0;
        best_distance_ = 0;
        nearest_(root_, pt, 0);
        return NearestNeighborResult{best_->point_, best_distance_, best_->value_};
    }

    /**
     * @brief performs a range query on the kdtree.
     * A range query takes a bounding box, and returns all points which intersect the bounding box.
     * @param query
     * @return std::optional<std::vector<RangeQueryResult>>
     */
    [[nodiscard]] auto range_query(const RangeQuery& query) const -> std::optional<std::vector<RangeQueryResult>> {
        const auto traverse = [this, &query](Node* root) -> std::vector<RangeQueryResult> {
            // https://slides.com/cos226/kd-tree-range-search/fullscreen#/1
            const auto [x_min, y_min, z_min] = query.min;
            const auto [x_max, y_max, z_max] = query.max;

            // is c between a and b
            // whether a < b or a > b does not matter
            const auto between = [](const auto a, const auto b, const auto c) {
                return ! ((c > a && c > b) || (c < a && c < b));
            };

            const auto between_x = [&, x_min, x_max](const float x) { return between(x_min, x_max, x); };
            const auto between_y = [&, y_min, y_max](const float y) { return between(y_min, y_max, y); };
            const auto between_z = [&, z_min, z_max](const float z) { return between(z_min, z_max, z); };
            const auto in_query_rectangle = [&](const Point& p) {
                return between_x(p.x()) && between_y(p.y()) && between_z(p.z());
            };

            struct kdtree3_search_bbx {
                float x_min, x_max, y_min, y_max, z_min, z_max;
                float operator[](std::size_t index) const {
                    switch (index) {
                        case 0:
                            return x_min;
                        case 1:
                            return x_max;
                        case 2:
                            return y_min;
                        case 3:
                            return y_max;
                        case 4:
                            return z_min;
                        case 5:
                            return z_max;
                        default:
                            throw std::invalid_argument("index out of range");
                    }
                }
            };  // kdtree3_search_bbx

            const auto kdtree3_search_bbx_intersects_query_rectangle = [=](const kdtree3_search_bbx& bbx) {
                const auto in_cuboid = [=](float x, float y, float z) {
                    return between(x_min, x_max, x) && between(y_min, y_max, y) && between(z_min, z_max, z);
                };

                // const auto [x_min, x_max, y_min, y_max, z_min, z_max] = bbx;
                // a 3D cuboid has 8 vertices, and at least one of them has to intersect.
                return in_cuboid(bbx.x_min, bbx.y_min, bbx.z_min) || in_cuboid(bbx.x_max, bbx.y_max, bbx.z_max) ||
                       in_cuboid(bbx.x_min, bbx.y_max, bbx.z_min) || in_cuboid(bbx.x_max, bbx.y_max, bbx.z_min) ||
                       in_cuboid(bbx.x_max, bbx.y_min, bbx.z_min) || in_cuboid(bbx.x_min, bbx.y_min, bbx.z_max) ||
                       in_cuboid(bbx.x_min, bbx.y_max, bbx.z_max) || in_cuboid(bbx.x_max, bbx.y_min, bbx.z_max);
            };

            auto acc = std::vector<RangeQueryResult>();

            const auto traverse_rec = [&](Node* root, kdtree3_search_bbx bbx, std::size_t index, const auto&& impl) {
                // base case 1
                // return if current node is null
                if (root == nullptr) {
                    return;
                }

                // base case 2
                // return if bounding box does not intersect query rectangle
                if (! kdtree3_search_bbx_intersects_query_rectangle(bbx)) {
                    return;
                }

                // check if the point belongs to the query rectangle, and if so append it to
                // the result list
                if (in_query_rectangle(root->point_)) {
                    acc.emplace_back(root->point_, root->value_);
                }

                auto left_bbx = bbx;
                auto right_bbx = bbx;
                switch (index % 3) {
                    case 0:  // x
                        left_bbx.x_max = root_->point_.x();
                        right_bbx.x_min = root_->point_.x();
                        break;
                    case 1:  // y
                        left_bbx.y_max = root_->point_.y();
                        right_bbx.y_min = root_->point_.y();
                        break;
                    case 2:  // y
                        left_bbx.z_max = root_->point_.z();
                        right_bbx.z_min = root_->point_.z();
                        break;
                };

                // go left
                impl(root->left_, left_bbx, index + 1, impl);
                // go right
                impl(root->right_, right_bbx, index + 1, impl);
            };

            static constexpr float MIN = std::numeric_limits<float>::min();
            static constexpr float MAX = std::numeric_limits<float>::max();

            auto initial_search_bbx = kdtree3_search_bbx{
                MIN, MAX, MIN, MAX, MIN, MAX,
            };

            traverse_rec(root, initial_search_bbx, traverse_rec);
            return acc;
        };

        const auto points_in_range_query = traverse(root_);

        return points_in_range_query.empty() ? std::nullopt : points_in_range_query;
    }

   private:
    struct Node {
        Node(Point pt, const Value& value) : point_(std::move(pt)), value_(value), left_(nullptr), right_(nullptr) {}

        [[nodiscard]] auto distance(const Point& pt) const -> double { return (point_ - pt).norm(); }
        [[nodiscard]] auto get_element_in_dimension(int index) const -> coordinate_type {
            assert(0 <= index && index < 3);
            return point_(index);
        }

        Point point_;
        Value value_;
        Node* left_;
        Node* right_;
    };

    struct dimension_comparator {
        dimension_comparator(int index) : index_(index) {}
        bool operator()(const Node& node1, const Node& node2) const {
            return node1.get_element_in_dimension(index_) < node2.get_element_in_dimension(index_);
        }
        int index_;
    };

    Node* root_ = nullptr;
    Node* best_ = nullptr;
    double best_distance_ = 0;
    std::size_t n_visited_ = 0;
    std::vector<Node> nodes_{};

    auto make_tree_(std::size_t begin, std::size_t end, std::size_t index) -> Node* {
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

    auto nearest_(Node* root, const Point& point, std::size_t index) -> void {
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
