#pragma once

#include <vector>

namespace softgps {

std::vector<int> generate_ca_code(int prn);
std::vector<float> sampled_ca_code(int prn, double sample_rate_hz, int samples_per_ms);

} // namespace softgps
