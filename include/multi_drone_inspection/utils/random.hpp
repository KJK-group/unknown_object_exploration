#ifndef _MULTI_DRONE_INSPECTION_RANDOM_HPP_
#define _MULTI_DRONE_INSPECTION_RANDOM_HPP_

#include <cassert>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <random>

namespace mdi::utils::random {

using Eigen::AngleAxisf;
using Eigen::Vector2f;
using Eigen::Vector3f;

/**
 * @brief Returns a uniform random number between 0.0 and 1.0
 * imp ref: https://stackoverflow.com/a/36527160
 * @return float [0, 1]
 */
auto random01() -> float {
    static std::default_random_engine e;
    static std::uniform_real_distribution<> dis(
        0, std::nextafter(1.f, std::numeric_limits<float>::max()));
    return dis(e);
}

auto get_bias_rotation_interval(float bias) -> float {
#ifndef NDEBUG
    if (!(0.f < bias && bias < 1.f)) {
        throw std::invalid_argument("bias must be between 0 and 1");
    }
#endif
    auto biased_angle_interval = 2 * M_PI - (bias * 2 * M_PI);
    return (random01() * 2 - 1) * biased_angle_interval / 2;
}

auto sample_random_point_on_unit_circle_surface() -> Vector2f {
    auto theta = get_bias_rotation_interval(0.f);
    auto rotation = Eigen::Rotation2Df(theta);
    return rotation * Vector2f::UnitX();
}

auto sample_random_point_on_unit_circle_surface(Vector2f direction, float bias) -> Vector2f {
    auto theta = get_bias_rotation_interval(bias);
    auto rotation = Eigen::Rotation2Df(theta);
    return rotation * direction;
}

auto sample_random_point_inside_unit_circle() -> Vector2f {
    return sample_random_point_on_unit_circle_surface() * random01();
}

auto sample_random_point_on_inside_circle(Vector2f direction, float bias) -> Vector2f {
    return sample_random_point_on_unit_circle_surface(direction, bias) * random01();
}

/**
 * @brief
 * imp ref: https://datagenetics.com/blog/january32020/index.html
 * @return Vector3f
 */
// auto sample_random_point_inside_unit_sphere() -> Vector3f {
//     float x, y, z;
//     do {
//         // select random point in "eight cube" i.e. within [-1, 1]
//         x = random01() * 2 - 1;
//         y = random01() * 2 - 1;
//         z = random01() * 2 - 1;
//     } while (sqrt((x * x) + (y * y) + (z * z)) > 1);  // discard if not within unit sphere.
//     return Vector3f(x, y, z);
// }

//----------------------------------------------------------------

auto sample_random_point_on_unit_sphere_surface() -> Vector3f {
    return Eigen::Quaternionf::UnitRandom() * Vector3f::UnitX();
}

auto sample_random_point_on_unit_sphere_surface(Vector3f direction, float x_bias, float y_bias,
                                                float z_bias) -> Vector3f {
    direction.normalize();
    auto roll = get_bias_rotation_interval(x_bias);
    auto pitch = get_bias_rotation_interval(y_bias);
    auto yaw = get_bias_rotation_interval(z_bias);
    auto quat = AngleAxisf(roll, Vector3f::UnitX()) * AngleAxisf(pitch, Vector3f::UnitY()) *
                AngleAxisf(yaw, Vector3f::UnitZ());
    return quat * direction;
}

auto sample_random_point_on_unit_sphere_surface(Vector3f direction, float bias) -> Vector3f {
    return sample_random_point_on_unit_sphere_surface(direction, bias, bias, bias);
}

// -------------------------------

auto sample_random_point_inside_unit_sphere() -> Vector3f {
    auto radial_distance = random01();
    return sample_random_point_on_unit_sphere_surface() * radial_distance;
}

auto sample_random_point_inside_unit_sphere(Vector3f direction, float x_bias, float y_bias,
                                            float z_bias) -> Vector3f {
    auto radial_distance = random01();
    return sample_random_point_on_unit_sphere_surface(direction, x_bias, y_bias, z_bias) *
           radial_distance;
}

/**
 * @brief
 *
 * @param direction
 * @param bias Likelihood of sampling a point in the direction given.
 * bias = [0, 1]
 * @return Vector3f
 */
auto sample_random_point_inside_unit_sphere(Vector3f direction, float bias) -> Vector3f {
    return sample_random_point_inside_unit_sphere(direction, bias, bias, bias);
}

}  // namespace mdi::utils::random

#endif  // _MULTI_DRONE_INSPECTION_RANDOM_HPP_
