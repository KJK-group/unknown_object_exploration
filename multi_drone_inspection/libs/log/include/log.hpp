#ifndef _LOG_HPP_
#define _LOG_HPP_

#include <chrono>
#include <iostream>
#include <string>
#include <string_view>

namespace log {

class log final {
   public:
    static auto info(std::string_view msg, std::ostream& os = std::cerr) -> void {}

   private:
    static auto timestamp() -> void {}

};  // log

}  // namespace log

#endif  // _LOG_HPP_
