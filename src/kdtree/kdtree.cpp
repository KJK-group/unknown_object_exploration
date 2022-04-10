#include "kdtree/kdtree.hpp"

namespace kdtree {

template <typename coordinate_t, size_t dimensions>
std::ostream& operator<<(std::ostream& out, const point<coordinate_t, dimensions>& pt) {
    static_assert(std::is_floating_point_v<coordinate_t> || std::is_integral_v<coordinate_t>,
                  "coordinate_t must be an integer of floating point");
    out << '(';
    for (size_t i = 0; i < dimensions; ++i) {
        if (i > 0) out << ", ";
        out << pt.get_element_in_dimension(i);
    }
    out << ')';
    return out;
}
}  // namespace kdtree
