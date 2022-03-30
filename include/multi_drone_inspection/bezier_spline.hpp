#ifndef _MDI_BEZIER_SPLINE_HPP_
#define _MDI_BEZIER_SPLINE_HPP_

#include <tf2_eigen/tf2_eigen.h>

#include <boost/math/special_functions/factorials.hpp>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>

using boost::math::factorial;
using Eigen::Vector3f;
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

   private:
    vector<Vector3f> spline_points;
    vector<int> binomial_lut;
    float arc_length;
    float resolution;
    auto get_time(float distance) -> float;
    auto generate_binomial_lut(int n) -> void;
    auto binomial_coefficient(int n, int i) -> int;
};
}  // namespace mdi

#endif