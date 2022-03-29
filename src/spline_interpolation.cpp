#include <tf2_eigen/tf2_eigen.h>

#include <cmath>

class BezierSpline {
   public:
    BezierSpline(float*** points);
    auto get_point(float time) -> float;

   private:
    float** LUT;
    float arc_length;
    auto get_time(float distance) -> float;
};