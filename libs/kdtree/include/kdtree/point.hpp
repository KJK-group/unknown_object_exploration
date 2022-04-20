#ifndef _MULTI_DRONE_INSPECTION_POINT_HPP_
#define _MULTI_DRONE_INSPECTION_POINT_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <type_traits>
#include <vector>

namespace kdtree {
/**
 * Class for representing a point. coordinate_t must be a numeric type.
 */
template <typename coordinate_t, size_t dimensions>
class point {
    static_assert(std::is_floating_point_v<coordinate_t> || std::is_integral_v<coordinate_t>,
                  "coordinate_t must be an integer or floating point");

   public:
    point(std::array<coordinate_t, dimensions> c) : coordinates_(c) {}

    point(std::initializer_list<coordinate_t> list) {
        size_t n = std::min(dimensions, list.size());
        std::copy_n(list.begin(), n, coordinates_.begin());
    }
    /**
     * Returns the coordinate in the given dimension.
     *
     * @param index dimension index (zero based)
     * @return coordinate in the given dimension
     */
    auto get_element_in_dimension(size_t index) const -> coordinate_t {
        return coordinates_[index];
    }
    /**
     * Returns the distance squared from this point to another
     * point.
     *
     * @param pt another point
     * @return distance squared from this point to the other point
     */
    auto distance(const point& pt) const -> double {
        double dist = 0;
        for (size_t i = 0; i < dimensions; ++i) {
            double d = get_element_in_dimension(i) - pt.get_element_in_dimension(i);
            dist += d * d;
        }
        return dist;
    }

   private:
    std::array<coordinate_t, dimensions> coordinates_;
};  // class point

}  // namespace kdtree

#endif  // _MULTI_DRONE_INSPECTION_POINT_HPP_
