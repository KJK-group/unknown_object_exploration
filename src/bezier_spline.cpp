#include "multi_drone_inspection/bezier_spline.hpp"

#include <iostream>

namespace mdi {
//--------------------------------------------------------------------------------------------------
// Constructor; generates the spline the first time,
// calling generate_spline with `points` and `resolution`
BezierSpline::BezierSpline(vector<Vector3f> points, int resolution) {
    // generate the spline points
    generate_spline(points, resolution);
}

//--------------------------------------------------------------------------------------------------
// Generates the spline from the input points given in the vector `points`,
// at a time resolution of `resolution`
auto BezierSpline::generate_spline(vector<Vector3f> points, int resolution) -> void {
    this->resolution = resolution;
    auto n = points.size();
    // in case the binomial lookup table isn't big enough, generate a new one
    if (binomial_lut.size() != n) {
        generate_binomial_lut(n);
    }
    assert(points.size() == binomial_lut.size());

    // let t run through [0;1] with steps defined by the resolution
    for (int t = 0; t <= 1 * resolution; t += 1) {
        // sum up all bezier terms, using the explicit definition given by:
        // https://en.wikipedia.org/wiki/B%C3%A9zier_curve
        auto time = (float)t / (float)resolution;
        auto spline_point = Vector3f(0, 0, 0);
        std::cout << "time: " << time << std::endl;
        for (int i = 0; i < n; i++) {
            std::cout << "points[" << i << "] = [" << points[i](0) << " " << points[i](1) << " "
                      << points[i](2) << "]"
                      << "\t";
            std::cout << "binomial_lut[" << i << "] = " << binomial_lut[i] << "\t";

            auto w = binomial_lut[i] * pow(time, i) * pow(1 - time, n - i);

            std::cout << "w for " << i << " = " << w << std::endl;

            spline_point += w * points[i];
        }
        std::cout << "resulting point = [" << spline_point(0) << " " << spline_point(1) << " "
                  << spline_point(2) << "]" << std::endl;

        spline_points.push_back(spline_point);
    }
}

//--------------------------------------------------------------------------------------------------
// Generates the binomial LUT for a Bezier pline with `n` point
// This will be used to generate the spline
auto BezierSpline::generate_binomial_lut(int n) -> void {
    for (int i = 0; i < n; i++) {
        binomial_lut.push_back(mdi::utils::binomial_coefficient(n, i));
    }
}

auto BezierSpline::get_point_at_time(float time) -> Vector3f {
    assert(time <= 1.0 && time >= 0);
    return this->spline_points[(int)(this->spline_points.size() * time)];
}

auto BezierSpline::get_spline_points() -> vector<Vector3f> { return this->spline_points; }
}  // namespace mdi