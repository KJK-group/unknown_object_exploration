#include "mdi/bezier_spline.hpp"

#include <iostream>

namespace mdi {
//--------------------------------------------------------------------------------------------------
// Constructor; generates the spline the first time,
// calling generate_spline with `points` and `resolution`
BezierSpline::BezierSpline(vector<Vector3f> points, int resolution) {
    // std::cout << "Inside spline constructor" << std::endl;
    // generate the spline points
    generate_spline(points, resolution);
}

//--------------------------------------------------------------------------------------------------
// Generates the spline from the input points given in the vector `points`,
// at a time resolution of `resolution`
auto BezierSpline::generate_spline(vector<Vector3f> points, int resolution) -> void {
    // std::cout << "Inside spline generator" << std::endl;

    if (points.size() > 20) {
        points.resize(20);
    }
    this->input_points = points;
    this->resolution = resolution;
    this->size = points.size();
    this->offset = points.back();

    this->generate_offset_points();
    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " before binomial LUT generation" << '\n';

    // in case the binomial lookup table isn't big enough, generate a new one
    if (binomial_lut.size() != this->size) {
        // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " inside if" << '\n';
        this->generate_binomial_lut();
        // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " inside if after generation" << '\n';
        // for (auto& bc : this->binomial_lut) {
        //     std::cout << bc << std::endl;
        // }
    }
    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //           << " binomial: " << this->binomial_lut.size() << '\n';
    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //           << " binomial: " << points.size() << '\n';

    assert(points.size() == binomial_lut.size());

    // let t run through [0;resolution] with steps defined by the resolution
    for (int t_idx = 0; t_idx < this->resolution; t_idx += 1) {
        auto time = (float)t_idx / (float)resolution;
        auto spline_point = f(time);
        this->spline_points.push_back(spline_point);
    }

    this->generate_distance_lut();
}

//--------------------------------------------------------------------------------------------------
// Generates the spline from the input points given in the vector `points`,
// at a time resolution of `resolution`

// using iterator = std::vector<Vector3f>::iterator;

// auto BezierSpline::generate_spline(iterator begin, iterator end, int resolution) -> void {
//     std::cout << "Inside spline generator" << std::endl;
//     this->input_points = points;
//     this->resolution = resolution;
//     this->size = std::distance(begin, end);
//     // in case the binomial lookup table isn't big enough, generate a new one
//     if (binomial_lut.size() != this->size) {
//         this->generate_binomial_lut();
//     }
//     //assert(points.size() == binomial_lut.size());

//     for (; begin != end; ++begin) {

//     }

//     // let t run through [0;resolution] with steps defined by the resolution
//     for (int t_idx = 0; t_idx < this->resolution; t_idx += 1) {
//         auto time = (float)t_idx / (float)resolution;
//         auto spline_point = f(time);
//         this->spline_points.push_back(spline_point);
//     }

//     this->generate_distance_lut();
// }

//--------------------------------------------------------------------------------------------------
// Returns the point along the spline at time `t`
// Uses the binomial LUT to perform these calculation fast
auto BezierSpline::f(float t) -> Vector3f {
    // sum up all bezier terms, using the explicit definition given by:
    // https://en.wikipedia.org/wiki/B%C3%A9zier_curve
    auto spline_point = Vector3f(0, 0, 0);
    for (int i = 0; i < this->size; i++) {
        auto w = binomial_lut[i] * pow(t, i) * pow(1 - t, this->size - i);
        spline_point += w * this->offset_input_points[i];
    }
    return spline_point;
}

//--------------------------------------------------------------------------------------------------
// Translates spline input points to start at the origin
auto BezierSpline::generate_offset_points() -> void {
    // std::cout << "OFFSET POINTS" << std::endl;
    this->offset_input_points = vector<Vector3f>();
    for (auto& point : this->input_points) {
        auto offset_point = point - this->offset;
        // std::cout << "POINT: " << offset_point << std::endl;
        this->offset_input_points.emplace_back(offset_point);
    }
}

//--------------------------------------------------------------------------------------------------
// Approximates the arc length of the spline from the sum of euclidean distances
// between all the calculated points along the spline
auto BezierSpline::generate_distance_lut() -> void {
    // assume that all the points have been generated
    assert(this->spline_points.size() == this->resolution);
    // Approximate the arc length of the spline
    auto arc_length_sum = 0.f;
    // first entry into distance LUT at time 0
    this->distance_lut.push_back(arc_length_sum);
    // std::cout << "this->spline_points.size() = " << this->spline_points.size() << std::endl;
    for (int i = 0; i < this->spline_points.size(); i++) {
        // std::cout << "point[" << i << "](0): " << this->spline_points[i](0) << std::endl;
        // std::cout << "point[" << i << "](1): " << this->spline_points[i](1) << std::endl;
        // std::cout << "point[" << i << "](2): " << this->spline_points[i](2) << std::endl;
    }
    // std::cout << "resolution = " << this->resolution << std::endl;
    for (int t_idx = 1; t_idx < this->resolution; t_idx++) {
        // difference vector between every two points
        // std::cout << "iteration = " << t_idx << std::endl;
        auto diff = this->spline_points[t_idx - 1] - this->spline_points[t_idx];
        // std::cout << "diff.norm() = " << diff.norm() << std::endl;
        // euclidean norm of the difference vector
        arc_length_sum += diff.norm();
        // std::cout << "arc_length_sum = " << arc_length_sum << std::endl;
        // cumulative distance entry for ever time step
        this->distance_lut.push_back(arc_length_sum);
    }
    // expect that the distance LUT has as many entries as resolution
    assert(this->distance_lut.size() == this->resolution);
}

//--------------------------------------------------------------------------------------------------
// Generates the binomial LUT for a Bezier pline with `n` points
// This will be used to generate the spline
auto BezierSpline::generate_binomial_lut() -> void {
    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " before for loop" << '\n';
    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " size: " << this->size << '\n';

    for (int i = 0; i < this->size; i++) {
        // std::cout << "idx: " << i << std::endl;
        binomial_lut.push_back(mdi::utils::binomial_coefficient(this->size, i));
    }
    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " after for loop" << '\n';
}

//--------------------------------------------------------------------------------------------------
// Returns the closest point at the given `time`, where 0<=time<=1.
auto BezierSpline::get_point_at_time(float time) -> Vector3f {
    assert(time <= 1.0 && time >= 0);
    return this->spline_points[(int)(resolution * time)] + this->offset;
}

//--------------------------------------------------------------------------------------------------
// Returns the point at the `distance` along the spline
auto BezierSpline::get_point_at_distance(float distance) -> Vector3f {
    // assume distance is greater not negative
    assert(distance >= 0);
    // std::cout << "distance = " << distance << std::endl;
    distance = this->distance_lut[this->distance_lut.size() - 1] - distance;
    if (distance < 0) {
        distance = 0;
    }
    // std::cout << "distance = " << distance << std::endl;
    auto t_idx = this->get_time_idx(distance);
    // std::cout << "t_idx = " << t_idx << std::endl;

    // given that the found time idx is the last idx,
    // the given distance must have been >= arc length,
    // thus the last point is returned
    // std::cout << "this->resolution - 1 = " << this->resolution - 1 << std::endl;
    if (t_idx == this->resolution - 1) {
        return this->spline_points[t_idx];
    }
    // otherwise we want to interpolate between two time values,
    // the one given, which is the closest before the `distance`,
    // and the next one after the given `distance`

    // for (auto& d : this->distance_lut) {
    //     std::cout << "distance: " << d << std::endl;
    // }

    // distance range to map from
    auto distance_diff = this->distance_lut[t_idx + 1] - this->distance_lut[t_idx];
    // std::cout << "distance_diff = " << distance_diff << std::endl;
    // distance to map
    auto distance_diff_given = this->distance_lut[t_idx + 1] - distance;
    // std::cout << "distance_diff_given = " << distance_diff_given << std::endl;
    // mapping to time with fraction of the above two
    auto diff_frac = 1 - distance_diff_given / distance_diff;
    // std::cout << "diff_frac = " << diff_frac << std::endl;
    assert(diff_frac >= 0 && diff_frac <= 1);
    // normalise to [0;1]
    auto t = ((float)t_idx + diff_frac) / resolution;
    // std::cout << "t = " << t << "\n" << std::endl;

    return this->f(t) + this->offset;
}

//--------------------------------------------------------------------------------------------------
// Returns the closes time idx, the given `distance` comes after
// If `distance`<0 return minimum time idx; 0
// If `distance`>arc_length, return max time idx; resolution
auto BezierSpline::get_time_idx(float distance) -> int {
    auto found_t_idx = 0;
    for (int t_idx = 0; t_idx < this->resolution && this->distance_lut[t_idx] < distance; t_idx++) {
        found_t_idx = t_idx;
    }
    return found_t_idx;
}

//--------------------------------------------------------------------------------------------------
// Returns a vector of all points along the spline
auto BezierSpline::get_spline_points() -> vector<Vector3f> {
    auto copy = vector<Vector3f>(this->spline_points.size());
    std::transform(this->spline_points.begin(), this->spline_points.end(), std::back_inserter(copy),
                   [this](const Vector3f& point) { return point + this->offset; });
    return this->spline_points;  // copy;
}

//--------------------------------------------------------------------------------------------------
// Return the arc length of the spline,
// found at the last index of the distance LUT
auto BezierSpline::get_length() -> float {
    if (this->distance_lut.size() != 0) {
        return this->distance_lut[this->distance_lut.size() - 1];
    } else {
        return 0;
    }
}
}  // namespace mdi