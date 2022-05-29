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

// #include "Eigen/src/Geometry/AngleAxis.h"
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
/* template <typename T>
auto yaml(std::vector<T> xs, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto whitespace = std::string(indentation, ' ');
    for (const auto x : xs) {
        yaml(x, indentation + tabsize, tabsize);
    }
}
 */
constexpr inline auto rad2deg(double a) -> double { return a * 180.0 / M_PI; };
constexpr inline auto deg2rad(double a) -> double { return a * M_PI / 180.0; };

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
                break;
            case AngleEncoding::Radian:
                angle_ = a;
                break;
        }
    }

    static auto from_degrees(float a) -> Angle<encoding, min, max> {
        return Angle<encoding, min, max>{static_cast<float>(deg2rad(a))};
    }
    [[nodiscard]] auto as_degree() const -> float { return rad2deg(angle_); }
    [[nodiscard]] auto angle() const -> float { return angle_; }
};

template <AngleEncoding encoding, int min, int max>
auto yaml(const Angle<encoding, min, max> angle, int indentation = 0, int tabsize = 2)
    -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + "\n"; };
    const auto line2 = [&](const std::string& s, float f) {
        return whitespace + s + std::to_string(f) + "\n";
    };
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
           line(tab + "y: " + std::to_string(position.y())) +
           line(tab + "z: " + std::to_string(position.z())) +
           line(tab + "euclidian norm: " + std::to_string(position.norm()));
}

auto yaml(const Quaternion& quat, int indentation = 0, int tabsize = 2) -> std::string {
    const auto tab = std::string(tabsize, ' ');
    const auto whitespace = std::string(indentation, ' ');
    const auto line = [&](const std::string& s) { return whitespace + s + ": " + "\n"; };
    const auto field = [&](const std::string& s, float value) {
        return whitespace + tab + s + ": " + std::to_string(value) + "\n";
    };

    return line("Orientation") + field("x", quat.x()) + field("y", quat.y()) +
           field("z", quat.z()) + field("w", quat.w());
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
    M << 1.0f, 0.0f, 0.0f, 0.0f, std::cos(angle), std::sin(angle), 0.0f, -std::sin(angle),
        std::cos(angle);
    return M;
}

auto rotation_around_Y_axis(float angle) -> mat3x3 {
    mat3x3 M{};
    M << std::cos(angle), 0.0f, std::sin(angle), 0.0f, 1.0f, 0.0f, -std::sin(angle), 0.0f,
        std::cos(angle);
    return M;
}

auto rotation_around_Z_axis(float angle) -> mat3x3 {
    mat3x3 M{};
    M << std::cos(angle), -std::sin(angle), 0.0f, std::sin(angle), std::cos(angle), 0.0f, 0.0f,
        0.0f, 1.0f;
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
    return {std::acos(static_cast<float>(v.getX())), std::acos(static_cast<float>(v.getY())),
            std::acos(static_cast<float>(v.getZ()))};
}

auto transform(const tf2::Transform& tf, const vec3& v) -> vec3 {
    auto tmp = tf2::Vector3{v.x(), v.y(), v.z()};
    auto transformed = tf * tmp;
    return {static_cast<float>(transformed.getX()), static_cast<float>(transformed.getY()),
            static_cast<float>(transformed.getZ())};
}

struct SphericalCoordinate {
    double r, theta, psi;
};  // Sphe

inline auto to_cartesian_coordinate(const SphericalCoordinate& coord) -> vec3 {
    const auto [r, theta, psi] = coord;
    return {static_cast<float>(r * cos(psi) * sin(theta)),
            static_cast<float>(r * sin(psi) * sin(theta)), static_cast<float>(r * cos(theta))};
}

inline auto yaw_diff(const vec3& a, const vec3& b) -> double {
    return std::acos(a.normalized().dot(b.normalized()));
}

}  // namespace mdi::types
