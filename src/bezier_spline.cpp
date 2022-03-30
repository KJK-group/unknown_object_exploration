#include "multi_drone_inspection/bezier_spline.hpp"

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

    // let t run through [0;1] with steps defined by the resolution
    for (float t = 0; t < 1; t += 1 / resolution) {
        // sum up all bezier terms, using the explicit definition given by:
        // https://en.wikipedia.org/wiki/B%C3%A9zier_curve
        auto spline_point = Vector3f(0, 0, 0);
        for (int i = 0; i < n; i++) {
            spline_point += binomial_lut[i] * pow((1 - t), n) * points[i];
        }
        spline_points.push_back(spline_point);
    }
}

//--------------------------------------------------------------------------------------------------
// Returns the binomial coefficient for `n` choose `i`,
// using the factorial function from the boost library
auto BezierSpline::binomial_coefficient(int n, int i) -> int {
    return factorial<int>(n) / (factorial<int>(i) * factorial<int>(n - i));
}

//--------------------------------------------------------------------------------------------------
// Generates the binomial LUT for a Bezier pline with `n` point
// This will be used to generate the spline
auto BezierSpline::generate_binomial_lut(int n) -> void {
    for (int i = 0; i < n; i++) {
        binomial_lut.push_back(binomial_coefficient(n, i));
    }
}
}  // namespace mdi