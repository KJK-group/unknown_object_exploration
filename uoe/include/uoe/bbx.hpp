#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

#include "fov.hpp"
#include "uoe/fov.hpp"

namespace uoe {

namespace types {

struct BBX {
    double width{}, depth{}, height{};
    vec3 center{};

    BBX(vec3 min, vec3 max)
        : width{max.x() - min.x()},
          depth{max.y() - min.y()},
          height{max.z() - min.z()},
          center{static_cast<float>(min.x() + width / 2), static_cast<float>(min.y() + depth / 2),
                 static_cast<float>(min.z() + height / 2)} {
        assert(min.x() <= max.x() && min.y() <= max.y() && min.z() <= max.z());
    }

    [[nodiscard]] auto min() const -> vec3 {
        return {static_cast<float>(center.x() - width / 2),
                static_cast<float>(center.y() - depth / 2),
                static_cast<float>(center.z() - height / 2)};
    }
    [[nodiscard]] auto max() const -> vec3 {
        return {static_cast<float>(center.x() + width / 2),
                static_cast<float>(center.y() + depth / 2),
                static_cast<float>(center.z() + height / 2)};
    }

    /**
     * @brief firs four forms the upper plane and the last four forms the lower plane.
     *
     * @return std::array<vec3, 8>
     */
    [[nodiscard]] auto vertices() const -> std::array<vec3, 8> {
        const auto min_ = min();
        const auto max_ = max();

        const float x_min = min_.x();
        const float y_min = min_.y();
        const float z_min = min_.z();
        const float x_max = max_.x();
        const float y_max = max_.y();
        const float z_max = max_.z();

        const vec3 a = vec3{x_min, y_min, z_min};
        const vec3 b = vec3{x_max, y_min, z_min};
        const vec3 c = vec3{x_max, y_min, z_max};
        const vec3 d = vec3{x_min, y_min, z_max};

        const vec3 e = vec3{x_min, y_max, z_min};
        const vec3 f = vec3{x_max, y_max, z_min};
        const vec3 g = vec3{x_max, y_max, z_max};
        const vec3 h = vec3{x_min, y_max, z_max};

        return {a, b, c, d, e, f, g, h};
    }

    // chefs kiss
    [[nodiscard]] auto bounding_edges() const -> std::array<std::pair<vec3, vec3>, 12> {
        const auto vs = vertices();
        return {
            std::make_pair(vs[0], vs[1]), std::make_pair(vs[1], vs[2]),
            std::make_pair(vs[2], vs[3]), std::make_pair(vs[3], vs[0]),
            std::make_pair(vs[4], vs[5]), std::make_pair(vs[5], vs[6]),
            std::make_pair(vs[6], vs[7]), std::make_pair(vs[7], vs[4]),
            std::make_pair(vs[0], vs[4]), std::make_pair(vs[1], vs[5]),
            std::make_pair(vs[2], vs[6]), std::make_pair(vs[3], vs[7]),
        };
    }
};

}  // namespace types

using types::BBX;
using types::FoV;

auto compute_bbx(const FoV& fov) -> BBX {
    using uoe::types::vec3;
    auto vertices =
        std::vector<vec3>{fov.pose().position + fov.direction() * fov.depth_range().max};

    for (const auto& v : fov.bounding_direction_vectors()) {
        // const auto near_dist = fov.depth_range().min;
        // const auto far_dist = fov.depth_range().max;
        vertices.push_back(fov.pose().position + v * fov.depth_range().min);
        vertices.push_back(fov.pose().position + v * fov.depth_range().max);
    }

    const auto minmax_in_dim = [&](int dim) -> std::array<float, 2> {
        const auto compare_in_dim = [dim](const vec3& a, const vec3& b) { return a[dim] < b[dim]; };
        // const auto min = std::min_element(vertices.begin(), vertices.end(), compare_in_dim);
        // const auto max = std::max_element(vertices.begin(), vertices.end(), compare_in_dim);
        const auto [min, max] =
            std::minmax_element(vertices.begin(), vertices.end(), compare_in_dim);

        return {(*min)[dim], (*max)[dim]};
    };

    // const auto fmt_vec3 = [&](const vec3& v) -> std::string {
    //     return "[" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " +
    //            std::to_string(v.z()) + "]";
    // };

    // std::cout << "vertices" << '\n';
    // for (const auto& v : vertices) {
    //     // std::cout << uoe::types::yaml(v) << '\n';
    //     std::cout << fmt_vec3(v) << '\n';
    // }

    const auto [x_min, x_max] = minmax_in_dim(0);
    const auto [y_min, y_max] = minmax_in_dim(1);
    const auto [z_min, z_max] = minmax_in_dim(2);

    vec3 min{x_min, y_min, z_min};
    vec3 max{x_max, y_max, z_max};

    // std::cout << "min:" << '\n';
    // std::cout << fmt_vec3(min) << '\n';
    // std::cout << "max:" << '\n';
    // std::cout << fmt_vec3(max) << '\n';

    return BBX{min, max};
}

}  // namespace uoe
