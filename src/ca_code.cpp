#include "gnss/ca_code.hpp"

#include <array>
#include <stdexcept>

namespace gnss {
namespace {

constexpr std::array<std::pair<int, int>, 33> g2_taps{{
    {0, 0},   {2, 6},   {3, 7},   {4, 8},   {5, 9},   {1, 9},   {2, 10},  {1, 8},
    {2, 9},   {3, 10},  {2, 3},   {3, 4},   {5, 6},   {6, 7},   {7, 8},   {8, 9},
    {9, 10},  {1, 4},   {2, 5},   {3, 6},   {4, 7},   {5, 8},   {6, 9},   {1, 3},
    {4, 6},   {5, 7},   {6, 8},   {7, 9},   {8, 10},  {1, 6},   {2, 7},   {3, 8},
    {4, 9},
}};

int bit_to_chip(int bit) {
    return bit == 0 ? 1 : -1;
}

} // namespace

std::vector<int> generate_ca_code(int prn) {
    if (prn < 1 || prn > 32) {
        throw std::out_of_range("GPS C/A PRN must be in 1..32");
    }

    std::array<int, 10> g1{};
    std::array<int, 10> g2{};
    g1.fill(1);
    g2.fill(1);

    std::vector<int> code;
    code.reserve(1023);
    const auto [tap_a, tap_b] = g2_taps[static_cast<std::size_t>(prn)];

    for (int i = 0; i < 1023; ++i) {
        const int g2_out = g2[static_cast<std::size_t>(tap_a - 1)] ^ g2[static_cast<std::size_t>(tap_b - 1)];
        code.push_back(bit_to_chip(g1[9] ^ g2_out));

        const int g1_feedback = g1[2] ^ g1[9];
        const int g2_feedback = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];
        for (int j = 9; j > 0; --j) {
            g1[static_cast<std::size_t>(j)] = g1[static_cast<std::size_t>(j - 1)];
            g2[static_cast<std::size_t>(j)] = g2[static_cast<std::size_t>(j - 1)];
        }
        g1[0] = g1_feedback;
        g2[0] = g2_feedback;
    }

    return code;
}

std::vector<float> sampled_ca_code(int prn, double sample_rate_hz, int samples_per_ms) {
    const auto chips = generate_ca_code(prn);
    std::vector<float> sampled(static_cast<std::size_t>(samples_per_ms));
    constexpr double chips_per_ms = 1023.0;
    for (int i = 0; i < samples_per_ms; ++i) {
        const double t_ms = static_cast<double>(i) / sample_rate_hz * 1000.0;
        const int chip = static_cast<int>(t_ms * chips_per_ms) % 1023;
        sampled[static_cast<std::size_t>(i)] = static_cast<float>(chips[static_cast<std::size_t>(chip)]);
    }
    return sampled;
}

} // namespace gnss
