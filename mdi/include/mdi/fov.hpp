#pragma once

#include <array>

// #include "Eigen/src/Geometry/Quaternion.h"
#include "mdi/common_types.hpp"

namespace mdi::types {

auto above_plane3d(const vec3& pt, const vec3& normal, double d) -> bool {
    return pt.dot(normal) + d > 0.0;
}
auto below_plane3d(const vec3& pt, const vec3& normal, double d) -> bool {
    return pt.dot(normal) + d < 0.0;
}
auto in_plane3d(const vec3& pt, const vec3& normal, double d, double tolerance = 1e-6) -> bool {
    return -tolerance <= pt.dot(normal) + d && pt.dot(normal) + d <= tolerance;
}

struct Plane3D {
    vec3 normal{};
    double d = 0.0;
    Plane3D(vec3 n, vec3 pt_in_plane) : normal{n}, d{-pt_in_plane.dot(n)} {}

    auto above(const vec3& pt) const -> bool { return above_plane3d(pt, normal, d); };
    auto below(const vec3& pt) const -> bool { return below_plane3d(pt, normal, d); };
    auto in(const vec3& pt, double tolerance = 1e-6) const -> bool {
        return in_plane3d(pt, normal, d, tolerance);
    };
};

class FoV {
   private:
    Pose pose_;
    FoVAngle horizontal_, vertical_;
    DepthRange depth_range_;
    Position target_;

    // mat3x3 rot_left_, rot_right_, rot_up_, rot_down_;
    mat3x3 T;
    vec3 upper_right_, lower_right_, upper_left_, lower_left_;

    vec3 direction_towards_target_;

    // Plane3D right_, left_, above_, below_;

    // std::array<Plane3D, 4> planes_{};
    std::vector<Plane3D> planes_{};

   public:
    FoV(Pose p, const FoVAngle& h, const FoVAngle& v, const DepthRange& d, Position target)
        : pose_{std::move(p)},
          horizontal_{h},
          vertical_{v},
          depth_range_{d},
          target_{std::move(target)} {
        // pose_.position.normalize();
        const float h2 = horizontal_.as_degree() / 2.0f;
        const float v2 = vertical_.as_degree() / 2.0f;

        auto [i_basis, j_basis, k_basis] = [&] {
            // TODO: calculate in a different way
            // auto quat = Eigen::Quaterniond{pose_.orientation.w, pose_.orientation.x,
            //    pose_.orientation.y, pose_.orientation.z};

            vec3 dir = p.orientation * vec3{1, 0, 0};

            // mat3x3 pitch_rot;
            // pitch_rot = quat * vec3{r}

            // vec3 dir = (target - pose_.position).normalized();
            direction_towards_target_ = dir;
            // 3d plane ax + by + cz + d = 0
            double a = dir.x();
            double b = dir.y();
            double c = dir.z();
            // double d = -pos.dot(dir);
            double d = 0.0;

            // constraint: roll = 0
            // assume: pitch != +- 90 deg
            // 1. find point in plane
            double k = 0;
            auto [i, j] = [&] {
                // 0 - 2pi
                double yaw = std::atan2(dir.y(), dir.x());
                double pi = M_PI;
                if ((0 <= yaw && yaw <= pi / 4) || (3 * pi / 4 <= yaw && yaw <= 5 * pi / 4) ||
                    (7 * pi / 4 <= yaw && yaw <= 2 * pi)) {
                    double j = 10;
                    double i = (-b * j - c * k - d) / a;
                    return std::make_pair(i, j);
                } else {
                    double i = 10;
                    double j = (-a * i - c * k - d) / b;
                    return std::make_pair(i, j);
                }
            }();
            // 2. project to global xy plane
            // 3. point.normalize();
            vec3 foo = vec3{static_cast<float>(i), static_cast<float>(j), static_cast<float>(k)}
                           .normalized();

            // 4. find 3rd basis pos.cross(point)
            vec3 bar = dir.cross(foo);

            return std::make_tuple(dir, foo, bar);
        }();

        T.col(0) = i_basis;
        T.col(1) = j_basis;
        T.col(2) = k_basis;

        {
            mat3x3 m{};
            m = AngleAxis(h2, k_basis) * AngleAxis(-v2, j_basis);
            upper_right_ = m * direction_towards_target_;
        }
        {
            mat3x3 m{};
            m = AngleAxis(h2, k_basis) * AngleAxis(v2, j_basis);
            lower_right_ = m * direction_towards_target_;
        }
        {
            mat3x3 m{};
            m = AngleAxis(-h2, k_basis) * AngleAxis(-v2, j_basis);
            upper_left_ = m * direction_towards_target_;
        }
        {
            mat3x3 m{};
            m = AngleAxis(-h2, k_basis) * AngleAxis(v2, j_basis);
            lower_left_ = m * direction_towards_target_;
        }

        planes_.emplace_back(upper_right_.cross(lower_right_), pose().position);
        planes_.emplace_back(lower_right_.cross(lower_left_), pose().position);
        planes_.emplace_back(lower_left_.cross(upper_left_), pose().position);
        planes_.emplace_back(upper_left_.cross(upper_right_), pose().position);
    }

    [[nodiscard]] auto pose() const -> const Pose& { return pose_; }
    [[nodiscard]] auto direction() const -> vec3 { return direction_towards_target_; }
    [[nodiscard]] auto target() const -> const Position& { return target_; }

    [[nodiscard]] auto horizontal() const -> FoVAngle { return horizontal_; }
    [[nodiscard]] auto vertical() const -> FoVAngle { return vertical_; }
    [[nodiscard]] auto depth_range() const -> const DepthRange& { return depth_range_; }

    [[nodiscard]] auto upper_right() const -> vec3 { return upper_right_; }
    [[nodiscard]] auto lower_right() const -> vec3 { return lower_right_; }
    [[nodiscard]] auto lower_left() const -> vec3 { return lower_left_; }
    [[nodiscard]] auto upper_left() const -> vec3 { return upper_left_; }

    [[nodiscard]] auto transform(const vec3& v) const -> vec3 { return T * v + pose().position; }

    /**
     * @brief the order is clockwise starting upper right in the direction facing away from the
     * camera.
     *
     * @return std::array<vec3, 4>
     */
    [[nodiscard]] auto bounding_direction_vectors() const -> std::array<vec3, 4> {
        return {upper_right(), lower_right(), lower_left(), upper_left()};
    }

    /**
     * @brief the order is clockwise starting upper right in the direction facing away from the
     * camera. represented in the same frame as the Pose.
     * @return std::array<vec3, 4>
     */
    [[nodiscard]] auto far_plane_vertices() const -> std::array<vec3, 4> {
        return {T * pose_.position + upper_right() * depth_range_.max,
                T * pose_.position + lower_right() * depth_range_.max,
                T * pose_.position + lower_left() * depth_range_.max,
                T * pose_.position + upper_left() * depth_range_.max};
    }

    /**
     * @brief the order is clockwise starting upper right in the direction facing away from the
     * camera. represented in the same frame as the Pose.
     * @return std::array<vec3, 4>
     */
    [[nodiscard]] auto near_plane_vertices() const -> std::array<vec3, 4> {
        return {T * pose_.position + upper_right() * depth_range_.min,
                T * pose_.position + lower_right() * depth_range_.min,
                T * pose_.position + lower_left() * depth_range_.min,
                T * pose_.position + upper_left() * depth_range_.min};
    }

    using bounding_trapezoid_iter_cb = std::function<void(float x, float y, float z)>;

    auto bounding_trapezoid_iter(const float resolution, bounding_trapezoid_iter_cb cb) -> void {
        const float delta_d = resolution;
        const auto span = [delta_d](const vec3& a, const vec3& b, float d) -> int {
            return std::ceil((a * d - b * d).norm() / delta_d);
        };

        const auto x_span = [&](float d) -> int { return span(upper_left_, upper_right_, d); };
        const auto y_span = [&](float d) -> int { return span(upper_left_, lower_left_, d); };

        const auto even = [](int a) { return a % 2 == 0; };
        const auto d_near_plane = depth_range_.min;

        auto offset = vec2{0, 0};
        if (even(x_span(d_near_plane))) {
            offset.x() = -delta_d / 2.0f;
        }

        if (even(y_span(d_near_plane))) {
            offset.y() = delta_d / 2.0f;
        }
        // near plane
        const auto coordinate_gen = [&](int x, int y) -> std::array<vec2, 2> {
            return {
                offset + vec2{-static_cast<float>(x) * delta_d, static_cast<float>(y) * delta_d},
                offset - vec2{-static_cast<float>(x) * delta_d, static_cast<float>(y) * delta_d}};
        };

        for (float x = depth_range_.min; x <= depth_range_.max; x += delta_d) {
            // figure out array dims
            const auto [start, end] = coordinate_gen(x_span(x), y_span(x));
            for (float y = start.x(); y < end.x(); y += delta_d) {
                for (float z = start.y(); z > end.y(); z -= delta_d) {
                    const auto v = T * vec3{x, y, z} + pose_.position;
                    cb(v.x(), v.y(), v.z());
                }
            }
        }
    }

    [[nodiscard]] auto plane_normals() const -> std::array<vec3, 4> {
        return {upper_right_.cross(lower_right_), lower_right_.cross(lower_left_),
                lower_left_.cross(upper_left_), upper_left_.cross(upper_right_)};
    }

    auto inside_fov(const vec3& pt) const -> bool {
        const double dist_from_fov_to_pt = (pt - pose_.position).norm();
        return depth_range_.min <= dist_from_fov_to_pt && dist_from_fov_to_pt <= depth_range_.max &&
               std::all_of(planes_.begin(), planes_.end(),
                           [&](const Plane3D& plane) { return plane.below(pt); });
    }

    // auto normal_plane() const -> mat3x3 {}
    // auto near_plane() const -> Pose {  }
    // auto far_plane() const -> Pose {  }
};

auto yaml(const FoV& fov, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');

    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };

    return line("FoV:") + yaml(fov.pose(), indentation + tabsize) + line(tab + "horizontal:") +
           yaml(fov.horizontal(), indentation + tabsize + tabsize, tabsize) +
           line(tab + "vertical:") +
           yaml(fov.vertical(), indentation + tabsize + tabsize, tabsize) +
           yaml(fov.depth_range(), indentation + tabsize, tabsize) + line("target:") +
           yaml(fov.target(), indentation + tabsize, tabsize);
}

}  // namespace mdi::types
