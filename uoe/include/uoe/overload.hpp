#pragma once

#include <functional>
#include <variant>
template <typename... Ts>
struct Overload : Ts... {
    using Ts::operator()...;
};
// needed in std 17 but not in std 20 and forward
template <class... Ts>
Overload(Ts...) -> Overload<Ts...>;

// template <class... Ts, typename V = std::variant<Ts...>>
// template <typename T, class... Ts>
// auto match(std::variant<Ts...>, std::function<T(Ts)>...);

// auto match(std::variant<Ts...> v, Ts... patterns) {
//     return std::visit(Overload<Ts...>{patterns...}, v);
// }
