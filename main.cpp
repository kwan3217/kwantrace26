#include <iostream>

#include "kwantrace/kwantrace.hpp"

using kwantrace::cosd;

auto test_multiply(kwantrace::Position p, kwantrace::Direction d, Eigen::Matrix4d M) {
    return M*p+M*d;
}

int main() {
    auto lang = "C++";
    std::cout << "Hello and welcome to " << lang << "!\n";

    for (int i = 1; i <= 5; i++) {
        std::cout << "i = " << cosd(i) << std::endl;
    }

    return 0;
}