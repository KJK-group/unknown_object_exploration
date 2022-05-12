#pragma once

#include <tf2/LinearMath/Transform.h>

#include <array>
#include <cassert>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <ratio>
#include <string>
#include <utility>

#include "ros/macros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace mdi::types {

using vec2 = Eigen::Vector2f;
using vec3 = Eigen::Vector3f;
using vec4 = Eigen::Vector4f;

using mat2x2 = Eigen::Matrix2f;
using mat3x3 = Eigen::Matrix3f;
using mat4x4 = Eigen::Matrix4f;

template <int min, int max>
class Angle {
   private:
    float angle_;

   public:
    explicit Angle(float a) {
        assert(min <= a && a <= max);
        angle_ = a;
    }

    static auto from_degrees(float a) -> Angle<min, max> {
        return Angle<min, max>{a * static_cast<float>(M_PI) / 180.0f};
    }
    [[nodiscard]] auto as_degree() const -> float { return angle_ * 180.0f / static_cast<float>(M_PI); }

    [[nodiscard]] auto angle() const -> float { return angle_; }
};

template <int min, int max>
auto yaml(const Angle<min, max> angle, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };
    const auto line2 = [&](const std::string& s, float f) { return whitespace + s + std::to_string(f) + "\n"; };
    return line("Angle:") + line2(tab + "min: ", min) + line2(tab + "max: ", max) +
           line2(tab + "radian: ", angle.angle()) + line2(tab + "degrees: ", angle.as_degree());
}

using FoVAngle = Angle<0, 180>;
using RotationAngle = Angle<0, 360>;

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
    Orientation orientation;
};

auto yaml(const Position& position, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');

    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };
    return line("Position:") + line(tab + "x: " + std::to_string(position.x())) +
           line(tab + "y: " + std::to_string(position.y())) + line(tab + "z: " + std::to_string(position.z()));
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
    M << std::cos(angle), 0.0f, -std::sin(angle), 0.0f, 1.0f, 0.0f, std::sin(angle), 0.0f, std::cos(angle);
    return M;
}

auto rotation_around_Z_axis(float angle) -> mat3x3 {
    mat3x3 M{};
    M << std::cos(angle), std::sin(angle), 0.0f, -std::sin(angle), std::cos(angle), 0.0f, 0.0f, 0.0f, 1.0f;
    return M;
}

class FoV {
   private:
    Pose pose_;
    FoVAngle horizontal_, vertical_;
    DepthRange depth_range_;

    mat3x3 rot_left_, rot_right_, rot_up_, rot_down_;
    vec3 upper_right_, lower_right_, upper_left_, lower_left_;

    tf2::Transform tf_;

    [[nodiscard]] auto transform_(const vec3& v) const -> vec3 {
        auto tf2_v = tf2::Vector3{v.x(), v.y(), v.z()};
        const auto v_transformed = tf_ * tf2_v;
        return {v_transformed.getX(), v_transformed.getY(), v_transformed.getZ()};
    }

   public:
    FoV(Pose p, const FoVAngle& h, const FoVAngle& v, const DepthRange& d)
        : pose_{std::move(p)}, horizontal_{h}, vertical_{v}, depth_range_{d} {
        pose_.position.normalize();
        const float h2 = horizontal_.angle() / 2.0f;
        const float v2 = vertical_.angle() / 2.0f;
        rot_left_ = rotation_around_Y_axis(-h2);
        rot_right_ = rotation_around_Y_axis(h2);
        rot_up_ = rotation_around_X_axis(-v2);
        rot_down_ = rotation_around_X_axis(v2);

        // upper_right_ = rot_right_ * rot_up_ * pose_.position;
        // lower_right_ = rot_right_ * rot_down_ * pose_.position;
        // upper_left_ = rot_left_ * rot_up_ * pose_.position;
        // lower_left_ = rot_left_ * rot_down_ * pose_.position;

        // auto tf2_ur = tf2::Vector3{0, 0, 1};
        tf_.setOrigin(tf2::Vector3(pose_.position.x(), pose_.position.y(), pose_.position.z()));
        auto quaternion = tf2::Quaternion{};
        quaternion.setEuler(pose_.orientation.yaw(), pose_.orientation.pitch(), pose_.orientation.roll());
        tf_.setRotation(quaternion);

        {
            auto tf_tmp = tf2::Transform{};
            const auto rotate = [&](vec3& vec, float h, float v) {
                auto quaternion = tf2::Quaternion{};
                quaternion.setRPY(h, v, 0);
                tf_tmp.setRotation(quaternion);
                auto tf_v = tf2::Vector3{pose_.position.x(), pose_.position.y(), pose_.position.z()};
                auto tf_v_rotated = tf_tmp * tf_v;
                vec[0] = tf_v_rotated.getX();
                vec[1] = tf_v_rotated.getY();
                vec[2] = tf_v_rotated.getZ();
            };

            rotate(upper_left_, -h2, -v2);
            rotate(upper_right_, h2, -v2);
            rotate(lower_left_, -h2, v2);
            rotate(lower_right_, h2, v2);
        }
    }

    // static auto from_degrees(float h, float v, const Pose& p, const DepthRange& d) -> FoV {
    //     // assert(0.0f <= h <= 360.0f && 0.0f <= v <= 360.0f);
    //     return {p, FoVAngle::from_degrees(h), FoVAngle::from_degrees(v), d};
    // }
    // static auto from_radians(FoVAngle h, FoVAngle v, const Pose& p, const DepthRange& d) -> FoV {
    //     // assert(0.0f <= h && h <= M_2_PI && 0.0f <= v && v <= M_2_PI);
    //     return {p, h, v, d};
    // }

    [[nodiscard]] auto pose() const -> Pose { return pose_; }

    [[nodiscard]] auto direction() const -> vec3 { return transform_(vec3::UnitX()); }
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
    [[nodiscard]] auto compute_bounding_direction_vectors() const -> std::array<vec3, 4> {
        return {upper_right(), lower_right(), lower_left(), upper_left()};
    }

    /**
     * @brief the order is clockwise starting upper right in the direction facing away from the camera.
     * represented in the same frame as the Pose.
     * @return std::array<vec3, 4>
     */
    [[nodiscard]] auto compute_endpoints() const -> std::array<vec3, 4> {
        return {
            transform_(upper_right() * depth_range_.max + pose_.position),
            transform_(lower_right() * depth_range_.max + pose_.position),
            transform_(upper_left() * depth_range_.max + pose_.position),
            transform_(lower_left() * depth_range_.max + pose_.position),
        };
    }

    using bounding_trapezoid_iter_cb = std::function<void(float x, float y, float z)>;
    auto bounding_trapezoid_iter(const float resolution, bounding_trapezoid_iter_cb cb) -> void {
        const float delta_d = resolution;

        // const auto iterations = std::ceil((depth_range_.max - depth_range_.min) / delta_d);

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
            return {center + offset + vec2{-static_cast<float>(x) * delta_d, static_cast<float>(y) * delta_d},
                    center - offset - vec2{-static_cast<float>(x) * delta_d, static_cast<float>(y) * delta_d}};
        };

        for (float z = depth_range_.min; z <= depth_range_.max; z += delta_d) {
            // figure out array dims
            const auto [start, end] = coordinate_gen(x_span(z), y_span(z));
            for (float x = start.x(); x < end.x(); x += delta_d) {
                for (float y = start.y(); y > end.y(); y -= delta_d) {
                    auto v = tf2::Vector3{x, y, z};
                    const auto v_transformed = tf_ * v;

                    cb(v_transformed.getX(), v_transformed.getY(), v_transformed.getZ());
                }
            }
        }
    }

    // auto normal_plane() const -> mat3x3 {}
    // auto near_plane() const -> void {  }
    // auto far_plane() const -> void {  }
};  // namespace mdi::types

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
