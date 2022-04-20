#include "multi_drone_inspection/utils/math.hpp"

#include <functional>

namespace mdi::utils {
//--------------------------------------------------------------------------------------------------
// Return the factorial of `i`
auto factorial(unsigned int i) -> unsigned int {
    std::function<unsigned int(unsigned int, unsigned int)> factorial_rec;
    factorial_rec = [&factorial_rec](unsigned int i, int a) -> unsigned int {
        return i == 0 ? a : factorial_rec(--i, a * i);
    };
    return factorial_rec(i, 1);
}

//--------------------------------------------------------------------------------------------------
// Returns the binomial coefficient for `n` choose `i`,
// using the factorial function above
auto binomial_coefficient(int n, int i) -> int {
    return factorial(n) / (factorial(i) * factorial(n - i));
}
}  // namespace mdi::utils