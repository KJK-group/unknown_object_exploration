#pragma once

#include <tf2_eigen/tf2_eigen.h>

#include <cassert>
#include <cmath>
#include <utility>
#include <vector>

#include "mdi/utils/math.hpp"

namespace mdi::trajectory {

using Eigen::Vector3f;
// using mdi::utils::binomial_coefficient;
using std::abs;
using std::pair;
using std::pow;
using std::vector;

//--------------------------------------------------------------------------------------------------
// Generates the binomial LUT for a Bezier pline with `n` points
// This will be used to generate the spline
const int binomial_lut_size = 20;
constexpr auto generate_binomial_lut() -> std::array<float, binomial_lut_size> {
    auto lut = std::array<float, binomial_lut_size>{};
    for (int i = 0; i < binomial_lut_size; i++) {
        lut[i] = (mdi::utils::binomial_coefficient(binomial_lut_size, i));
    }
    return lut;
}
// static constexpr std::array<float, binomial_lut_size> binomial_lut{
//     mdi::utils::binomial_coefficient(binomial_lut_size, 0),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 1),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 2),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 3),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 4),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 5),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 6),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 7),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 8),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 9),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 10),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 11),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 12),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 13),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 14),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 15),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 16),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 17),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 18),
//     mdi::utils::binomial_coefficient(binomial_lut_size, 19)};

class BezierSpline {
   public:
    BezierSpline() {}
    BezierSpline(vector<Vector3f> points, int resolution = 40);
    auto generate_spline(vector<Vector3f> points, int resolution = 40) -> void;
    auto get_point_at_time(float time) -> Vector3f;
    auto get_point_at_distance(float distance) -> Vector3f;
    auto get_spline_points() -> vector<Vector3f>;
    auto get_length() -> float;
    auto f(float t) -> Vector3f;  // spline polynomial function

   private:
    auto generate_distance_lut() -> void;
    auto generate_offset_points() -> void;

    vector<Vector3f> input_points;         // idx: input point number, element: 3D input point
    vector<Vector3f> offset_input_points;  // idx: offset point number
    vector<Vector3f> spline_points;        // idx: output point number, element: 3D output point
    // static vector<int> binomial_lut;       // idx: input point number, element: binomial
    // coeffient
    static constexpr std::array<float, binomial_lut_size> binomial_lut = generate_binomial_lut();
    vector<float> distance_lut;  // idx: time, element: distance - arc length at last idx
    Vector3f offset;             // translation from origin
    int resolution;              // output size
    int size;                    // input size
    auto get_time_idx(float distance) -> int;
    // auto generate_binomial_lut() -> void;
};
// BezierSpline::binomial_lut = BezierSpline::generate_binomial_lut<20>();
}  // namespace mdi::trajectory
