#include "multi_drone_inspection/utils/math.hpp"

namespace mdi::utils {
//--------------------------------------------------------------------------------------------------
// Return the factorial of `i`
auto factorial(unsigned int i) -> uint64_t {
    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " inside factorial, before for loop" << '\n';

    std::function<uint64_t(unsigned int, uint64_t)> factorial_rec;
    factorial_rec = [&factorial_rec](unsigned int i, uint64_t a) -> uint64_t {
        // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " a = " << a << '\n';
        // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
        //   << " i = " << i << '\n';

        return i == 0 ? a : factorial_rec(--i, a * i);
    };

    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " inside factorial, after for loop" << '\n';

    return factorial_rec(i, 1);
}

//--------------------------------------------------------------------------------------------------
// Returns the binomial coefficient for `n` choose `i`,
// using the factorial function above
auto binomial_coefficient(int n, int i) -> int {
    // std::cerr << "[DEBUG] " << __FILE__ << ":" << __LINE__ << ": "
    //   << " inside binomial coefficient" << '\n';

    return (factorial(n) / (factorial(i) * factorial(n - i)));
}
}  // namespace mdi::utils