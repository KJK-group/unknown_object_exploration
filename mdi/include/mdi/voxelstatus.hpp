#pragma once

#include <octomap/octomap.h>

#include <variant>

#include "mdi/overload.hpp"
#include "octomap/octomap_types.h"

namespace mdi {

enum class VoxelStatus { Free, Occupied, Unknown };

// auto  = Overload {
// [](char) { return "char"; },
// [](int) { return "int"; },
// [](unsigned int) { return "unsigned int"; },
// [](long int) { return "long int"; },
// [](long long int) { return "long long int"; },
// [](auto) { return "unknown type"; },
// };

}  // namespace mdi

struct Free {};
struct Occupied {
    octomap::point3d center;
};
struct Unknown {
    octomap::point3d center;
};

using Voxel = std::variant<Unknown, Occupied, Free>;
