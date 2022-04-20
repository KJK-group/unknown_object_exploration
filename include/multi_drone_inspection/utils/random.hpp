#ifndef _MULTI_DRONE_INSPECTION_RANDOM_HPP_
#define _MULTI_DRONE_INSPECTION_RANDOM_HPP_

#include <cassert>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <random>

namespace mdi::utils::random {

using Eigen::AngleAxisf;
using Eigen::Vector2f;
using Eigen::Vector3d;
using Eigen::Vector3f;

struct random_point_generator {
    random_point_generator(float min, float max)
        : engine_(std::random_device()()), distribution_(min, max) {}
    //   distribution_(min, std::nextafter(max, std::numeric_limits<float>::max())) {}

    Eigen::Vector3d operator()() {
        float x = distribution_(engine_);
        float y = distribution_(engine_);
        float z = distribution_(engine_);
        return {x, y, z};
    }

    // std::mt19937 engine_;
    std::default_random_engine engine_;
    std::uniform_real_distribution<float> distribution_;

    auto random01() -> float {  //    std::random_device rd;
        return distribution_(engine_);
    }

    auto get_bias_rotation_interval(float bias) -> float {
#ifndef NDEBUG
        if (!(0.f < bias && bias < 1.f)) {
            throw std::invalid_argument("bias must be between 0 and 1");
        }
#endif
        auto biased_angle_interval = 2 * M_PI - (bias * 2 * M_PI);
        return (random01() * 2.0 - 1.0) * biased_angle_interval / 2.0;
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
        auto quat = Eigen::Quaternionf::UnitRandom();
        auto unit = Vector3f{random01(), random01(), random01()}.normalized();
        // std::cout << "   (quat * unit).normalized() " << (quat * unit).normalized() << std::endl;
        return (quat * unit).normalized();
    }

    auto sample_random_point_on_unit_sphere_surface(Vector3f direction, float x_bias, float y_bias,
                                                    float z_bias) -> Vector3f {
        direction.normalize();
        // std::cout << "direction: " << direction << std::endl;
        auto roll = get_bias_rotation_interval(x_bias);
        auto pitch = get_bias_rotation_interval(y_bias);
        auto yaw = get_bias_rotation_interval(z_bias);
        // std::cout << "roll interval is " << std::to_string(roll) << " pitch interval is "
        //   << std::to_string(pitch) << " yaw interval is " << std::to_string(yaw) << std::endl;
        Eigen::Matrix3f quat;
        quat = AngleAxisf(roll, Vector3f::UnitX()) * AngleAxisf(pitch, Vector3f::UnitY()) *
               AngleAxisf(yaw, Vector3f::UnitZ());

        auto rotated = quat * direction;
        // std::cout << "rotated " << rotated << std::endl;
        // std::cout << "rotated again" << rotated << std::endl;
        return rotated;
    }

    auto sample_random_point_on_unit_sphere_surface(Vector3f direction, float bias) -> Vector3f {
        return sample_random_point_on_unit_sphere_surface(direction, bias, bias, bias);
    }

    // -------------------------------

    auto sample_random_point_inside_unit_sphere() -> Vector3f {
        auto radial_distance = random01();
        // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " radial_distance" << radial_distance << '\n';

        auto a = sample_random_point_on_unit_sphere_surface() * radial_distance;
        // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " a" << a << '\n';

        return a;
    }

    auto sample_random_point_inside_unit_sphere(Vector3f direction, float x_bias, float y_bias,
                                                float z_bias) -> Vector3f {
        auto radial_distance = random01();
        radial_distance *= 1.f;
        // DO NOT REMOVE LINE BELOW. RADIAL_DISTANCE IS NOT RANDOM IF NOT
        // std::cout << "radial distance: " << radial_distance << std::endl;
        auto v = sample_random_point_on_unit_sphere_surface(direction, x_bias, y_bias, z_bias);
        // std::cout << "v is " << v << std::endl;
        return radial_distance * v;
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
};

// /**
//  * @brief Returns a uniform random number between 0.0 and 1.0
//  * imp ref: https://stackoverflow.com/a/36527160
//  * @return float [0, 1]
//  */
// auto random01() -> float {  //    std::random_device rd;
//     //    std::mt19937 gen(rd());
//     // static std::mt19937 mt(123);
//     // std::mt19937 generator(
//     //   std::chrono::steady_clock::now().time_since_epoch().count());
//     static auto gen = std::mt19937{std::random_device{}()};
//     // static auto gen =
//     std::mt19937{std::chrono::steady_clock::now().time_since_epoch().count()}; static
//     std::uniform_real_distribution<float> dis(
//         0, std::nextafter(1.f, std::numeric_limits<float>::max()));
//     return dis(gen);
// }

// auto get_bias_rotation_interval(float bias) -> float {
// #ifndef NDEBUG
//     if (!(0.f < bias && bias < 1.f)) {
//         throw std::invalid_argument("bias must be between 0 and 1");
//     }
// #endif
//     auto biased_angle_interval = 2 * M_PI - (bias * 2 * M_PI);
//     return (random01() * 2 - 1) * biased_angle_interval / 2;
// }

// auto sample_random_point_on_unit_circle_surface() -> Vector2f {
//     auto theta = get_bias_rotation_interval(0.f);
//     auto rotation = Eigen::Rotation2Df(theta);
//     return rotation * Vector2f::UnitX();
// }

// auto sample_random_point_on_unit_circle_surface(Vector2f direction, float bias) -> Vector2f {
//     auto theta = get_bias_rotation_interval(bias);
//     auto rotation = Eigen::Rotation2Df(theta);
//     return rotation * direction;
// }

// auto sample_random_point_inside_unit_circle() -> Vector2f {
//     return sample_random_point_on_unit_circle_surface() * random01();
// }

// auto sample_random_point_on_inside_circle(Vector2f direction, float bias) -> Vector2f {
//     return sample_random_point_on_unit_circle_surface(direction, bias) * random01();
// }

// /**
//  * @brief
//  * imp ref: https://datagenetics.com/blog/january32020/index.html
//  * @return Vector3f
//  */
// // auto sample_random_point_inside_unit_sphere() -> Vector3f {
// //     float x, y, z;
// //     do {
// //         // select random point in "eight cube" i.e. within [-1, 1]
// //         x = random01() * 2 - 1;
// //         y = random01() * 2 - 1;
// //         z = random01() * 2 - 1;
// //     } while (sqrt((x * x) + (y * y) + (z * z)) > 1);  // discard if not within unit sphere.
// //     return Vector3f(x, y, z);
// // }

// //----------------------------------------------------------------

// auto sample_random_point_on_unit_sphere_surface() -> Vector3f {
//     auto quat = Eigen::Quaternionf::UnitRandom();
//     auto unit = Vector3f{random01(), random01(), random01()}.normalized();
//     // std::cout << "    " << unit << std::endl;
//     return (quat * unit).normalized();

//     // return Eigen::Quaternionf::UnitRandom() * Vector3f::UnitRandom();
// }

// auto sample_random_point_on_unit_sphere_surface(Vector3f direction, float x_bias, float y_bias,
//                                                 float z_bias) -> Vector3f {
//     direction.normalize();
//     // std::cout << "direction: " << direction << std::endl;
//     auto roll = get_bias_rotation_interval(x_bias);
//     auto pitch = get_bias_rotation_interval(y_bias);
//     auto yaw = get_bias_rotation_interval(z_bias);
//     // std::cout << "roll interval is " << std::to_string(roll) << " pitch interval is "
//     //   << std::to_string(pitch) << " yaw interval is " << std::to_string(yaw) << std::endl;
//     Eigen::Matrix3f quat;
//     quat = AngleAxisf(roll, Vector3f::UnitX()) * AngleAxisf(pitch, Vector3f::UnitY()) *
//            AngleAxisf(yaw, Vector3f::UnitZ());

//     auto rotated = quat * direction;
//     // std::cout << "rotated " << rotated << std::endl;
//     // std::cout << "rotated again" << rotated << std::endl;
//     return rotated;
// }

// auto sample_random_point_on_unit_sphere_surface(Vector3f direction, float bias) -> Vector3f {
//     return sample_random_point_on_unit_sphere_surface(direction, bias, bias, bias);
// }

// // -------------------------------

// auto sample_random_point_inside_unit_sphere() -> Vector3f {
//     auto radial_distance = random01();
//     return sample_random_point_on_unit_sphere_surface() * radial_distance;
// }

// auto sample_random_point_inside_unit_sphere(Vector3f direction, float x_bias, float y_bias,
//                                             float z_bias) -> Vector3f {
//     auto radial_distance = random01();
//     radial_distance *= 1.f;
//     // DO NOT REMOVE LINE BELOW. RADIAL_DISTANCE IS NOT RANDOM IF NOT
//     // std::cout << "radial distance: " << radial_distance << std::endl;
//     auto v = sample_random_point_on_unit_sphere_surface(direction, x_bias, y_bias, z_bias);
//     // std::cout << "v is " << v << std::endl;
//     return radial_distance * v;
// }

// /**
//  * @brief
//  *
//  * @param direction
//  * @param bias Likelihood of sampling a point in the direction given.
//  * bias = [0, 1]
//  * @return Vector3f
//  */
// auto sample_random_point_inside_unit_sphere(Vector3f direction, float bias) -> Vector3f {
//     return sample_random_point_inside_unit_sphere(direction, bias, bias, bias);
// }

}  // namespace mdi::utils::random

#endif  // _MULTI_DRONE_INSPECTION_RANDOM_HPP_
