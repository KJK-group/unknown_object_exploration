#pragma once

#include <cassert>
#include <cstddef>
#include <iostream>
#include <ostream>
#include <vector>

namespace mdi::types::graph {

struct Edge {
    std::size_t from, to;
};

// first index is vertex id (a), second is vertex id (b) of a vertex that it forms
// an edge with (a,b)
using Graph = std::vector<std::vector<std::size_t>>;

auto visited(const Graph& g, const Edge& e) -> bool {
    const auto [u, v] = e;

    assert(u != v);
    assert(0 <= u && u < g.size());

    const auto& edges = g[u];

    for (const auto connected_vertex : edges) {
        if (v == connected_vertex) {
            return true;
        }
    }
    return false;
};

// returns true if the edge does not exist, it then marks the edge as visited.
auto mark_as_visited(Graph& g, const Edge& e) -> bool {
    const auto [u, v] = e;

    assert(u != v);
    assert(0 <= u && u < g.size());

    auto& edges = g[u];

    for (const auto connected_vertex : edges) {
        if (v == connected_vertex) {
            return false;
        }
    }
    edges.push_back(v);
    return true;
};
auto print_graph(const Graph& g, std::ostream& os = std::cout) {
    for (std::size_t u = 0; u < g.size(); ++u) {
        os << "[ ";
        for (std::size_t v = 0; v < g[u].size(); ++v) {
            os << "(" << u << ", " << v << "), ";
        }
        os << " ]" << std::endl;
    }
    os << "\n\n";
};

}  // namespace mdi::types::graph
