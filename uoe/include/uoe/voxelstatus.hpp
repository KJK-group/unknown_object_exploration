#pragma once

#include <octomap/octomap.h>

#include <variant>

#include "uoe/overload.hpp"
#include "octomap/octomap_types.h"

namespace uoe {

enum class VoxelStatus { Free, Occupied, Unknown };

// auto  = Overload {
// [](char) { return "char"; },
// [](int) { return "int"; },
// [](unsigned int) { return "unsigned int"; },
// [](long int) { return "long int"; },
// [](long long int) { return "long long int"; },
// [](auto) { return "unknown type"; },
// };

}  // namespace uoe

struct Free {};
struct Occupied {
    octomap::point3d center;
};
struct Unknown {
    octomap::point3d center;
};

using Voxel = std::variant<Unknown, Occupied, Free>;
