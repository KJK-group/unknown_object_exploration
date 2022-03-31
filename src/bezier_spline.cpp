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
    for (int t = 0; t <= resolution; t += 1) {
        // sum up all bezier terms, using the explicit definition given by:
        // https://en.wikipedia.org/wiki/B%C3%A9zier_curve
        auto time = (float)t / (float)resolution;
        auto spline_point = Vector3f(0, 0, 0);
        for (int i = 0; i < n; i++) {
            auto w = binomial_lut[i] * pow(time, i) * pow(1 - time, n - i);
            spline_point += w * points[i];
        }

        spline_points.push_back(spline_point);
    }
}

//--------------------------------------------------------------------------------------------------
// Approximates the arc length of the spline from the sum of euclidean distances
// between all the calculated points along the spline
auto BezierSpline::approximate_arc_length() -> void {
    // Approximate the arc length of the spline
    auto arc_length_sum = 0;
    // first entry into distance LUT at time 0
    this->distance_lut.push_back(arc_length_sum);
    for (int t = 1; t < spline_points.size(); t++) {
        // difference vector between every two points
        auto diff = spline_points[t - 1] - spline_points[t];
        // euclidean norm of the difference vector
        arc_length_sum += diff.norm();
        // cumulative distance entry for ever time step
        this->distance_lut.push_back(arc_length_sum);
    }
    this->arc_length = arc_length_sum;
}

//--------------------------------------------------------------------------------------------------
// Generates the binomial LUT for a Bezier pline with `n` points
// This will be used to generate the spline
auto BezierSpline::generate_binomial_lut(int n) -> void {
    for (int i = 0; i < n; i++) {
        binomial_lut.push_back(mdi::utils::binomial_coefficient(n, i));
    }
}

//--------------------------------------------------------------------------------------------------
// Returns the closest point at the given `time`, where 0<=time<=1.
auto BezierSpline::get_point_at_time(float time) -> Vector3f {
    assert(time <= 1.0 && time >= 0);
    return this->spline_points[(int)(this->spline_points.size() * time)];
}
//--------------------------------------------------------------------------------------------------
// Returns the point at the `distance` along the spline
auto BezierSpline::get_point_at_distance(float distance) -> Vector3f {
    // have a distance vs time LUT
    // generate this in arc length approximation method

    // find the two distances from the LUT, that the `distance` lies withing
    // then remap the distance the two distances, to the distance between the two time values
    // then take the lowest time value and add the mapped time value between the two time value
    // of the two distances closest to the given `distance`
}

//--------------------------------------------------------------------------------------------------
// Returns a vector of all points along the spline
auto BezierSpline::get_spline_points() -> vector<Vector3f> { return this->spline_points; }
}  // namespace mdi