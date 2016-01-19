#include <ceres_slam/utils.h>

#include <vector>
#include <string>
#include <sstream>

namespace ceres_slam {

std::vector<std::string> split(const std::string str, const char del) {
    std::stringstream ss(str); // Copy the string into a stream
    std::vector<std::string> tokens;
    std::string tok;

    while(getline(ss, tok, del)) {
        tokens.push_back(tok);
    }

    return tokens;
}

} // namespace ceres_slam
