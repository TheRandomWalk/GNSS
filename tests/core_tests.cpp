#include "softgps/ca_code.hpp"

#include <cassert>
#include <numeric>

int main() {
    const auto prn1 = softgps::generate_ca_code(1);
    assert(prn1.size() == 1023);
    const int sum = std::accumulate(prn1.begin(), prn1.end(), 0);
    assert(sum == -1);

    const auto sampled = softgps::sampled_ca_code(1, 2.046e6, 2046);
    assert(sampled.size() == 2046);
    return 0;
}
