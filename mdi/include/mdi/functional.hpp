#pragma once

#include <algorithm>
#include <vector>

namespace functional {

auto map(const auto& iteratable, auto fn) -> std::vector<decltype(*(iteratable.begin()))> {
    auto begin = iteratable.begin();
    auto end = iteratable.end();
    auto result = std::vector<decltype(*begin)>(std::distance(begin, end));
    std::transform(begin, end, std::back_inserter(result), fn);
    return result;
}

}  // namespace functional
