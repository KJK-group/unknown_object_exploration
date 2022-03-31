#ifndef _MULTI_DRONE_INSPECTION_BEZIER_SPLINE_HPP_
#define _MULTI_DRONE_INSPECTION_BEZIER_SPLINE_HPP_

#include <tf2_eigen/tf2_eigen.h>

#include <cassert>
#include <cmath>
#include <utility>
#include <vector>

#include "multi_drone_inspection/utils/math.hpp"

using Eigen::Vector3f;
// using mdi::utils::binomial_coefficient;
using std::abs;
using std::pair;
using std::pow;
using std::vector;

namespace mdi {
// Generates necessary LUTs for a Bezier spline given a list of points
// and a time resolution
class BezierSpline {
   public:
    BezierSpline(vector<Vector3f> points, int resolution = 20);
    auto generate_spline(vector<Vector3f> points, int resolution = 20) -> void;
    auto get_point_at_time(float time) -> Vector3f;
    auto get_point_at_distance(float distance) -> Vector3f;
    auto get_spline_points() -> vector<Vector3f>;

   private:
    vector<Vector3f> input_points;   // idx: input point number, element: 3D input point
    vector<Vector3f> spline_points;  // idx: output point number, element: 3D output point
    vector<int> binomial_lut;        // idx: input point number, element: binomial coeffient
    vector<float> distance_lut;      // idx: time, element: distance - arc length at last idx
    int resolution;
    int size;
    auto f(float t) -> Vector3f;  // spline polynomial function
    auto get_time_idx(float distance) -> float;
    auto generate_binomial_lut() -> void;
    auto generate_distance_lut() -> void;
};
}  // namespace mdi

#endif  // _MULTI_DRONE_INSPECTION_BEZIER_SPLINE_HPP_