#pragma once

#include <math.h>
#include <tf2/LinearMath/Transform.h>

#include <array>
#include <cassert>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <functional>
#include <iostream>
#include <ratio>
#include <string>
#include <utility>

#include "Eigen/src/Geometry/AngleAxis.h"
#include "ros/macros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace mdi::types {

using std::cos, std::sin;
using vec2 = Eigen::Vector2f;
using vec3 = Eigen::Vector3f;
using vec4 = Eigen::Vector4f;

using mat2x2 = Eigen::Matrix2f;
using mat3x3 = Eigen::Matrix3f;
using mat4x4 = Eigen::Matrix4f;
using AngleAxis = Eigen::AngleAxisf;

using Quaternion = Eigen::Quaternionf;

inline auto quaternion_from_two_vectors(const vec3& u, const vec3& v) -> Quaternion {
    return Quaternion::FromTwoVectors(u, v);
}

// TODO: finish this
template <typename T>
auto yaml(std::vector<T> xs, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto whitespace = std::string(indentation, ' ');
    for (const auto x : xs) {
        yaml(x, indentation + tabsize, tabsize);
    }
}

constexpr inline auto rad2deg(double a) -> double { return a * 180.0f / static_cast<float>(M_PI); };
constexpr inline auto deg2rad(double a) -> double { return a * static_cast<float>(M_PI) / 180.0f; };

enum class AngleEncoding { Radian, Degree };

template <AngleEncoding encoding, int min, int max>
class Angle {
   private:
    float angle_;  // rad

   public:
    explicit Angle(float a) {
        assert(min <= a && a <= max);
        switch (encoding) {
            case AngleEncoding::Degree:
                angle_ = deg2rad(a);
            case AngleEncoding::Radian:
                angle_ = a;
        }
    }

    static auto from_degrees(float a) -> Angle<encoding, min, max> { return Angle<encoding, min, max>{deg2rad(a)}; }
    [[nodiscard]] auto as_degree() const -> float { return rad2deg(angle_); }
    [[nodiscard]] auto angle() const -> float { return angle_; }
};

template <AngleEncoding encoding, int min, int max>
auto yaml(const Angle<encoding, min, max> angle, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };
    const auto line2 = [&](const std::string& s, float f) { return whitespace + s + std::to_string(f) + "\n"; };
    return line("Angle:") + line2(tab + "min: ", min) + line2(tab + "max: ", max) +
           line2(tab + "radian: ", angle.angle()) + line2(tab + "degrees: ", angle.as_degree());
}

using FoVAngle = Angle<AngleEncoding::Degree, 0, 180>;
using RotationAngle = Angle<AngleEncoding::Degree, 0, 360>;

using Position = vec3;

class Orientation {
    RotationAngle r, p, y;

   public:
    Orientation(float roll, float pitch, float yaw) : r{roll}, p{pitch}, y{yaw} {}
    [[nodiscard]] auto roll() const -> float { return r.angle(); }
    [[nodiscard]] auto pitch() const -> float { return p.angle(); }
    [[nodiscard]] auto yaw() const -> float { return y.angle(); }
    [[nodiscard]] auto rpy() const -> vec3 { return {roll(), pitch(), yaw()}; }
};

struct Pose {
    Position position{};
    // Orientation orientation;
    Quaternion orientation;
};

auto yaml(const Position& position, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');

    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };
    return line("Position:") + line(tab + "x: " + std::to_string(position.x())) +
           line(tab + "y: " + std::to_string(position.y())) + line(tab + "z: " + std::to_string(position.z())) +
           line(tab + "euclidian norm: " + std::to_string(position.norm()));
}

auto yaml(const Quaternion& quat, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + ": " + "\n"; };
    const auto field = [&](const std::string& s, float value) {
        return whitespace + tab + s + ": " + std::to_string(value) + "\n";
    };

    return line("Position") + field("x", quat.x()) + field("y", quat.y()) + field("z", quat.z()) + field("w", quat.w());
}

auto yaml(const Orientation& orientation, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');

    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };

    return line("Orientation:") + line(tab + "roll: " + std::to_string(orientation.roll())) +
           line(tab + "pitch: " + std::to_string(orientation.pitch())) +
           line(tab + "yaw: " + std::to_string(orientation.yaw()));
}

auto yaml(const Pose& pose, int indentation = 0, int tabsize = 2) -> std::string {
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };

    return line("Pose:") + yaml(pose.position, indentation + tabsize, tabsize) +
           yaml(pose.orientation, indentation + tabsize, tabsize);
}

struct State {
    State(float x, float y, float z, float yaw) : position{x, y, z}, yaw{yaw} {}

    Position position{};
    RotationAngle yaw;
};

auto yaml(const State& state, int indentation = 0, int tabsize = 2) -> std::string {
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };
    return line("State:") + yaml(state.position, indentation + tabsize, tabsize) +
           yaml(state.yaw, indentation + tabsize, tabsize);
}

/**
 * @brief depth is measued in meters
 *
 */
struct DepthRange {
    explicit DepthRange(float min_, float max_) {
        assert(0.0f <= min_ && min_ < max_);
        min = min_;
        max = max_;
    }
    float min{};
    float max{};
};

auto yaml(const DepthRange& range, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');

    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };

    return line("DepthRange:") + line(tab + "min: " + std::to_string(range.min)) +
           line(tab + "max: " + std::to_string(range.max));
}

auto rotation_around_X_axis(float angle) -> mat3x3 {
    mat3x3 M{};
    M << 1.0f, 0.0f, 0.0f, 0.0f, std::cos(angle), std::sin(angle), 0.0f, -std::sin(angle), std::cos(angle);
    return M;
}

auto rotation_around_Y_axis(float angle) -> mat3x3 {
    mat3x3 M{};
    M << std::cos(angle), 0.0f, std::sin(angle), 0.0f, 1.0f, 0.0f, -std::sin(angle), 0.0f, std::cos(angle);
    return M;
}

auto rotation_around_Z_axis(float angle) -> mat3x3 {
    mat3x3 M{};
    M << std::cos(angle), -std::sin(angle), 0.0f, std::sin(angle), std::cos(angle), 0.0f, 0.0f, 0.0f, 1.0f;
    return M;
}

struct RPY {
    double r, p, y;
};  // RPY

auto angle_of_vector(vec3 v) -> RPY {
    v.normalize();
    return {std::acos(v.x()), std::acos(v.y()), std::acos(v.z())};
}
auto angle_of_vector(tf2::Vector3 v) -> RPY {
    v.normalize();
    return {std::acos(v.getX()), std::acos(v.getY()), std::acos(v.getZ())};
}

auto transform(const tf2::Transform& tf, const vec3& v) -> vec3 {
    auto tmp = tf2::Vector3{v.x(), v.y(), v.z()};
    auto transformed = tf * tmp;
    return {transformed.getX(), transformed.getY(), transformed.getZ()};
}

struct SphericalCoordinate {
    double r, theta, psi;
};  // Sphe

inline auto to_cartesian_coordinate(const SphericalCoordinate& coord) -> vec3 {
    const auto [r, theta, psi] = coord;
    return {r * cos(psi) * sin(theta), r * sin(psi) * sin(theta), r * cos(theta)};
}

inline auto yaw_diff(const vec3& a, const vec3& b) -> double { return std::acos(a.normalized().dot(b.normalized())); }

class FoV {
   private:
    Pose pose_;
    FoVAngle horizontal_, vertical_;
    DepthRange depth_range_;
    Position target_;

    mat3x3 rot_left_, rot_right_, rot_up_, rot_down_;
    mat3x3 T;
    vec3 upper_right_, lower_right_, upper_left_, lower_left_;

    vec3 direction_towards_target_;

    tf2::Transform tf_;

    // [[nodiscard]] auto transform_(const vec3& v) const -> vec3 {
    //     auto tf2_v = tf2::Vector3{v.x(), v.y(), v.z()};
    //     const auto v_transformed = tf_ * tf2_v;
    //     return {v_transformed.getX(), v_transformed.getY(), v_transformed.getZ()};
    // }

   public:
    FoV(Pose p, const FoVAngle& h, const FoVAngle& v, const DepthRange& d, Position target)
        : pose_{std::move(p)}, horizontal_{h}, vertical_{v}, depth_range_{d}, target_{std::move(target)} {
        // pose_.position.normalize();
        const float h2 = horizontal_.angle() / 2.0f;
        const float v2 = vertical_.angle() / 2.0f;

        auto [i_basis, j_basis, k_basis] = [&] {
            vec3 dir = (target - pose_.position).normalized();
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
            vec3 foo = vec3{i, j, k}.normalized();
            // 3. point.normalize();

            // 4. find 3rd basis pos.cross(point)
            vec3 bar = dir.cross(foo);

            return std::make_tuple(dir, foo, bar);
        }();

        // const float r = 1.0f;
        // const float theta = v2;
        // const float psi = h2;
        // const auto sc = SphericalCoordinate{r, theta, psi};
        // {
        //     const vec3 cc = to_cartesian_coordinate(sc);
        //     const float x = cc.x();
        //     const float y = cc.y();
        //     const float z = cc.z();
        //     upper_right_ = vec3(x, y, z);
        //     lower_right_ = vec3(x, y, -z);
        //     upper_left_ = vec3(x, -y, z);
        //     lower_left_ = vec3(x, -y, -z);
        // }

        rot_left_ = rotation_around_Z_axis(-h2);
        rot_right_ = rotation_around_Z_axis(h2);
        rot_up_ = rotation_around_X_axis(-v2);
        rot_down_ = rotation_around_X_axis(v2);

        // const vec3 up = vec3{0, 0, 1};
        // direction_towards_target_ = (target_ - pose_.position).normalized();

        // const vec3 gaze = direction_towards_target_;
        // T.col(2) = gaze;
        // T.col(0) = gaze.cross(up);
        // // T.col(2) = up;
        // const auto [roll, pitch, yaw] = angle_of_vector(direction_towards_target_);
        // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //           << " roll " << roll << " pitch " << pitch << " yaw " << yaw << std::endl;

        // const float yaw = yaw_diff(vec3{1, 0, 0}, direction_towards_target_);

        // T = rotation_around_Z_axis(yaw);

        // mat3x3 Rx90;
        // Rx90 << 1.0f, 0.0f, 0.0f, 0.0f, cos(M_PI_2), sin(M_PI_2), 0.0f, -sin(M_PI_2), cos(M_PI_2);
        // mat3x3 Rz90 = rotation_around_Z_axis(M_PI_2);

        // direction_towards_target_ = (target_ - pose_.position).normalized();
        // const vec3 i_basis = direction_towards_target_;
        // // ensure j_basis orthonormal to i_basis by rotating 90 degrees around
        // const vec3 j_basis = Rx90 * i_basis;
        // // use cross product to find third orthogonal basis vector.
        // const vec3 k_basis = i_basis.cross(j_basis).normalized();
        // T forms a orthonormal basis where i_basis is the direction of from -> to, and j_basis and k_basis
        // span the plane to which i_basis is a normal vector.
        T.col(0) = i_basis;
        T.col(1) = j_basis;
        T.col(2) = k_basis;

        // mat3x3 Rx90;
        // Rx90 << 1.0f, 0.0f, 0.0f, 0.0f, cos(M_PI_2), sin(M_PI_2), 0.0f, -sin(M_PI_2), cos(M_PI_2);
        // mat3x3 Rz90 = rotation_around_Z_axis(M_PI_2);

        // direction_towards_target_ = (target_ - pose_.position).normalized();
        // const vec3 i_basis = direction_towards_target_;
        // // ensure j_basis orthonormal to i_basis by rotating 90 degrees around
        // const vec3 j_basis = Rx90 * i_basis;
        // // use cross product to find third orthogonal basis vector.
        // const vec3 k_basis = i_basis.cross(j_basis).normalized();
        // // T forms a orthonormal basis where i_basis is the direction of from -> to, and j_basis and k_basis
        // // span the plane to which i_basis is a normal vector.
        // T.col(0) = i_basis;
        // T.col(1) = j_basis;
        // T.col(2) = k_basis;

        // const mat3x3 Rx = rotation_around_X_axis(roll);
        // const mat3x3 Ry = rotation_around_Y_axis(pitch);
        // const mat3x3 Rz = rotation_around_Z_axis(yaw);
        // T = Rz * Ry * Rx;

        // const vec3 i_basis = Rt.col(0);
        // const vec3 j_basis = Rt.col(1);
        // const vec3 k_basis = Rt.col(2);

        // auto q = tf2::Quaternion{};
        // q.setRPY(r, p, y);
        // auto t = tf2::Transform(q);

        // upper_right_ = /* rot_up_ * */ rot_right_ * direction_towards_target_;
        // lower_right_ = /* rot_down_ * */ rot_right_ * direction_towards_target_;
        // upper_left_ = /* rot_up_ * */ rot_left_ * direction_towards_target_;
        // lower_left_ = /* rot_down_ * */ rot_left_ * direction_towards_target_;

        // upper_right_ = rot_up_ * direction_towards_target_;
        // lower_right_ = rot_down_ * direction_towards_target_;
        // upper_left_ = rot_up_ * direction_towards_target_;
        // lower_left_ = rot_down_ * direction_towards_target_;

        // upper_right_ = rot_right_ * rot_up_ * (T * direction_towards_target_);
        // lower_right_ = rot_right_ * rot_down_ * (T * direction_towards_target_);
        // upper_left_ = rot_left_ * rot_up_ * (T * direction_towards_target_);
        // lower_left_ = rot_left_ * rot_down_ * (T * direction_towards_target_);

        {
            mat3x3 m{};
            auto q = Quaternion{};
            m = /*AngleAxis(0.25 * M_PI, vec3::UnitX()) * */ AngleAxis(h2, k_basis) * AngleAxis(-v2, j_basis);
            // q.toRotationMatrix() *
            upper_right_ = m * direction_towards_target_;
        }

        {
            mat3x3 m{};
            m = /*AngleAxis(0.25 * M_PI, k_basis) * */ AngleAxis(h2, k_basis) * AngleAxis(v2, j_basis);
            lower_right_ = m * direction_towards_target_;
        }
        {
            mat3x3 m{};
            m = /*AngleAxis(0.25 * M_PI, k_basis) * */ AngleAxis(-h2, k_basis) * AngleAxis(-v2, j_basis);
            upper_left_ = m * direction_towards_target_;
        }
        {
            mat3x3 m{};
            m = /*AngleAxis(0.25 * M_PI, k_basis) * */ AngleAxis(-h2, k_basis) * AngleAxis(v2, j_basis);
            lower_left_ = m * direction_towards_target_;
        }

        // upper_right_ = rot_right_ * rot_up_ * direction_towards_target_;
        // lower_right_ = rot_right_ * rot_down_ * direction_towards_target_;
        // upper_left_ = rot_left_ * rot_up_ * direction_towards_target_;
        // lower_left_ = rot_left_ * rot_down_ * direction_towards_target_;

        // auto tf2_ur = tf2::Vector3{0, 0, 1};
        // tf_.setOrigin(tf2::Vector3(pose_.position.x(), pose_.position.y(), pose_.position.z()));
        // auto quaternion = tf2::Quaternion{};
        // quaternion.setRPY(pose_.orientation.roll(), pose_.orientation.pitch(), pose_.orientation.yaw());
        // std::cout << "quaternion: " << '\n';
        // std::cout << quaternion.getX() << " " << quaternion.getY() << " " << quaternion.getZ() << " "
        //           << quaternion.getW() << '\n';
        // tf_.setRotation(quaternion);

        // {
        //     auto tf_tmp = tf2::Transform{};
        //     const auto rotate = [&](vec3& vec, float h, float v) {
        //         auto quaternion = tf2::Quaternion{};
        //         quaternion.setRPY(h, v, 0);
        //         tf_tmp.setRotation(quaternion);
        //         auto tf_v = tf2::Vector3{pose_.position.x(), pose_.position.y(), pose_.position.z()};
        //         auto tf_v_rotated = tf_tmp * tf_v;
        //         vec[0] = tf_v_rotated.getX();
        //         vec[1] = tf_v_rotated.getY();
        //         vec[2] = tf_v_rotated.getZ();
        //     };

        //     rotate(upper_left_, -h2, -v2);
        //     rotate(upper_right_, h2, -v2);
        //     rotate(lower_left_, -h2, v2);
        //     rotate(lower_right_, h2, v2);
        // }
    }

    [[nodiscard]] auto pose() const -> const Pose& { return pose_; }
    [[nodiscard]] auto direction() const -> vec3 { return direction_towards_target_; }
    [[nodiscard]] auto horizontal() const -> FoVAngle { return horizontal_; }
    [[nodiscard]] auto vertical() const -> FoVAngle { return vertical_; }
    [[nodiscard]] auto depth_range() const -> const DepthRange& { return depth_range_; }

    [[nodiscard]] auto upper_right() const -> vec3 { return upper_right_; }
    [[nodiscard]] auto lower_right() const -> vec3 { return lower_right_; }
    [[nodiscard]] auto lower_left() const -> vec3 { return lower_left_; }
    [[nodiscard]] auto upper_left() const -> vec3 { return upper_left_; }

    /**
     * @brief the order is clockwise starting upper right in the direction facing away from the camera.
     *
     * @return std::array<vec3, 4>
     */
    [[nodiscard]] auto bounding_direction_vectors() const -> std::array<vec3, 4> {
        return {upper_right(), lower_right(), lower_left(), upper_left()};
    }

    /**
     * @brief the order is clockwise starting upper right in the direction facing away from the camera.
     * represented in the same frame as the Pose.
     * @return std::array<vec3, 4>
     */
    // [[nodiscard]] auto endpoints() const -> std::array<vec3, 4> {
    //     return {
    //         transform_(upper_right() * depth_range_.max + pose_.position),
    //         transform_(lower_right() * depth_range_.max + pose_.position),
    //         transform_(upper_left() * depth_range_.max + pose_.position),
    //         transform_(lower_left() * depth_range_.max + pose_.position),
    //     };
    // }

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
        const auto center = vec2{0, 0};
        // near plane
        const auto coordinate_gen = [&](int x, int y) -> std::array<vec2, 2> {
            return {/*center + */ offset + vec2{-static_cast<float>(x) * delta_d, static_cast<float>(y) * delta_d},
                    /*center -*/ offset - vec2{-static_cast<float>(x) * delta_d, static_cast<float>(y) * delta_d}};
        };

        for (float x = depth_range_.min; x <= depth_range_.max; x += delta_d) {
            // figure out array dims
            const auto [start, end] = coordinate_gen(x_span(x), y_span(x));
            for (float z = start.x(); z < end.x(); z += delta_d) {
                for (float y = start.y(); y > end.y(); y -= delta_d) {
                    const auto v = T * vec3{x, y, z} + pose_.position;
                    cb(v.x(), v.y(), v.z());
                }
            }
        }
    }

    // auto normal_plane() const -> mat3x3 {}
    // auto near_plane() const -> Pose {  }
    // auto far_plane() const -> Pose {  }
};

auto yaml(FoV& fov, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');

    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };

    return line("FoV:") + yaml(fov.pose(), indentation + tabsize) + line(tab + "horizontal:") +
           yaml(fov.horizontal(), indentation + tabsize + tabsize, tabsize) + line(tab + "vertical:") +
           yaml(fov.vertical(), indentation + tabsize + tabsize, tabsize) +
           yaml(fov.depth_range(), indentation + tabsize, tabsize);
}

}  // namespace mdi::types
