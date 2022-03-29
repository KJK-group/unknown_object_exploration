#include <tf2_eigen/tf2_eigen.h>

#include <cmath>
#include <utility>
#include <vector>

using Eigen::Vector3f;
using std::pair;
using std::vector;

class BezierSpline {
   public:
    BezierSpline(vector<Vector3f> points);
    auto get_point(float time) -> float;

   private:
    vector<pair<float, float>> pointsLUT;
    float arc_length;
    auto get_time(float distance) -> float;
};