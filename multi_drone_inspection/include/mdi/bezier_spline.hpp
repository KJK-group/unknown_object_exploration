#ifndef _MULTI_DRONE_INSPECTION_BEZIER_SPLINE_HPP_
#define _MULTI_DRONE_INSPECTION_BEZIER_SPLINE_HPP_

#include <tf2_eigen/tf2_eigen.h>

#include <cassert>
#include <cmath>
#include <utility>
#include <vector>

#include "mdi/utils/math.hpp"

namespace mdi {

using Eigen::Vector3f;
// using mdi::utils::binomial_coefficient;
using std::abs;
using std::pair;
using std::pow;
using std::vector;
// Generates necessary LUTs for a Bezier spline given a list of points
// and a time resolution
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
    vector<Vector3f> input_points;         // idx: input point number, element: 3D input point
    vector<Vector3f> offset_input_points;  // idx: offset point number
    vector<Vector3f> spline_points;        // idx: output point number, element: 3D output point
    vector<int> binomial_lut;              // idx: input point number, element: binomial coeffient
    vector<float> distance_lut;            // idx: time, element: distance - arc length at last idx
    Vector3f offset;                       // translation from origin
    int resolution;                        // output size
    int size;                              // input size
    auto get_time_idx(float distance) -> int;
    auto generate_binomial_lut() -> void;
    auto generate_distance_lut() -> void;
    auto generate_offset_points() -> void;
};
}  // namespace mdi

#endif  // _MULTI_DRONE_INSPECTION_BEZIER_SPLINE_HPP_