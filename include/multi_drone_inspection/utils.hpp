#include <cassert>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <random>

using Eigen::Vector2f;
using Eigen::Vector3f;

namespace mpi::utils::random {

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

auto sample_random_point_on_unit_circle_surface() -> Vector2f {
    auto theta = random01() * 2 * M_PI;
    auto rotation = Eigen::Rotation2Df(theta);
    auto radial_distance = 1.f;
    return Vector3f(radial_distance, 0.f) * rotation;
}

auto sample_random_point_inside_unit_circle() -> Vector2f {
    return sample_random_point_on_unit_circle_surface() * random01();
}

/**
 * @brief
 * imp ref: https://datagenetics.com/blog/january32020/index.html
 * @return Vector3f
 */
auto sample_random_point_inside_unit_sphere() -> Vector3f {
    float x, y, z;
    do {
        // select random point in "eight cube" i.e. within [-1, 1]
        x = random01() * 2 - 1;
        y = random01() * 2 - 1;
        z = random01() * 2 - 1;
    } while (sqrt((x * x) + (y * y) + (z * z)) > 1);  // discard if not within unit sphere.
    return Vector3f(x, y, z);
}

// FIX:
auto sample_random_point_on_unit_sphere_surface() -> Vector3f {
    auto point = sample_random_point_inside_unit_sphere();
    return point.normalized();
}

}  // namespace mpi::utils::random
