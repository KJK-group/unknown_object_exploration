#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "mdi/utils/math.hpp"
#include "mdi/utils/random.hpp"
auto n_runs = 1e2;
auto n_points = 1e5;
constexpr auto r = 10;

auto measure(std::function<void()> f) -> void {
    using std::cout;
    using std::endl;
    const auto start = std::chrono::steady_clock::now();
    f();
    const auto end = std::chrono::steady_clock::now();

    cout << "Elapsed time in nanoseconds: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
         << " ns" << endl;

    cout << "Elapsed time in microseconds: "
         << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " µs" << endl;

    cout << "Elapsed time in milliseconds: "
         << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << endl;

    cout << "Elapsed time in seconds: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
         << " sec";
}

auto measure_average(std::function<void()> f, std::size_t runs) -> void {
    using std::cout;
    using std::endl;

    auto measurements = std::vector<std::int64_t>();

    for (std::size_t i = 0; i < runs; ++i) {
        const auto start = std::chrono::steady_clock::now();
        f();
        const auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        measurements.push_back(std::move(elapsed));
    }

    auto mean = std::accumulate(measurements.begin(), measurements.end(), 0) / measurements.size();
    std::cout << "mean average elapsed time(milliseconds): " << mean << std::endl;
}

auto fmt_vec3(const Eigen::Vector3f& v) -> std::string {
    return "[ " + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " + std::to_string(v.z()) + " ]";
}

//...

int main(int argc, char const* argv[]) {
    if (argc == 2) {
        n_points = atoi(argv[1]);
    } else if (argc == 3) {
        n_points = atoi(argv[1]);
        n_runs = atoi(argv[2]);
    }
    std::cout << "n_points: " << n_points << std::endl;
    std::cout << "n_runs: " << n_runs << std::endl;

    const auto direction = Eigen::Vector3f{1, 0.8, 0.3}.normalized();
    const auto bounding_sphere_center = Eigen::Vector3f::Ones() * 5.f;
    auto points = std::vector<Eigen::Vector3f>(n_points);
    for (size_t i = 0; i < n_points; ++i) {
        points[i] =
            (mdi::utils::random::sample_random_point_inside_unit_sphere(direction, 0) * r + bounding_sphere_center);
    }

    measure_average(
        [&]() {
            auto query_point =
                mdi::utils::random::sample_random_point_inside_unit_sphere(direction, 0) * r + bounding_sphere_center;

            auto best_distance = std::numeric_limits<float>::max();
            auto t1_best_distance = std::numeric_limits<float>::max();
            auto t2_best_distance = std::numeric_limits<float>::max();
            auto it = points.begin();
            auto t1 = std::thread([&]() {
                std::for_each(points.begin(), points.begin() + points.size() / 2, [&](const auto& point) {
                    auto distance = mdi::utils::squared_distance(query_point, point);
                    if (distance < t1_best_distance) {
                        t1_best_distance = distance;
                    }
                });
            });
            auto t2 = std::thread([&]() {
                std::for_each(points.begin() + points.size() / 2 + 1, points.end(), [&](const auto& point) {
                    auto distance = mdi::utils::squared_distance(query_point, point);
                    if (distance < t2_best_distance) {
                        t2_best_distance = distance;
                    }
                });
            });

            // for (const auto& p : points) {
            //     auto distance = mdi::utils::squared_distance(query_point, p);
            //     if (distance < best_distance) {
            //         best_distance = distance;
            //     }
            // }
            t1.join();
            t2.join();

            best_distance = std::min(t1_best_distance, t2_best_distance);

            std::cout << std::fixed << std::setprecision(4);
            std::cout << "best distance: for " << fmt_vec3(query_point) << " is " << std::setw(5) << best_distance
                      << std::endl;
        },
        n_runs);

    // for (int i = 0; i < n_runs; ++i) {
    //     measure([&]() {
    //         auto query_point =
    //             mdi::utils::random::sample_random_point_inside_unit_sphere(direction, 0) * r +
    //             bounding_sphere_center;

    //         auto best_distance = std::numeric_limits<float>::max();

    //         for (const auto& p : points) {
    //             auto distance = mdi::utils::squared_distance(query_point, p);
    //             if (distance < best_distance) {
    //                 best_distance = distance;
    //             }
    //         }
    //         std::cout << "best distance: for " << fmt_vec3(query_point) << " is " <<
    //         best_distance
    //                   << std::endl;
    //     });
    // }

    return 0;
}
