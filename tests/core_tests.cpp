#include "gnss/acquisition.hpp"
#include "gnss/ca_code.hpp"
#include "gnss/math.hpp"
#include "gnss/pvt.hpp"
#include "gnss/tracking.hpp"

#include <cassert>
#include <cmath>
#include <complex>
#include <numeric>
#include <vector>

namespace {

void test_ca_code() {
    const auto prn1 = gnss::generate_ca_code(1);
    assert(prn1.size() == 1023);
    const int sum = std::accumulate(prn1.begin(), prn1.end(), 0);
    assert(sum == -1);

    const auto sampled = gnss::sampled_ca_code(1, 2.046e6, 2046);
    assert(sampled.size() == 2046);
}

void test_acquisition_and_tracking() {
    constexpr double rate = 1.023e6;
    const auto code = gnss::generate_ca_code(1);
    std::vector<std::complex<float>> samples;
    samples.reserve(4092);
    for (int ms = 0; ms < 4; ++ms) {
        for (int chip = 0; chip < 1023; ++chip) {
            samples.emplace_back(static_cast<float>(code[static_cast<std::size_t>(chip)]), 0.0F);
        }
    }

    gnss::AcquisitionConfig config;
    config.sample_rate_hz = rate;
    config.doppler_min_hz = 0.0;
    config.doppler_max_hz = 0.0;
    config.doppler_step_hz = 500.0;
    config.threshold = 2.0;
    const auto results = gnss::acquire(samples, config, 1, 1);
    assert(results.size() == 1);
    assert(results.front().detected);
    assert(results.front().prn == 1);

    gnss::TrackingConfig tracking_config;
    tracking_config.sample_rate_hz = rate;
    tracking_config.max_epochs = 4;
    gnss::TrackingState state;
    state.prn = 1;
    state.code_phase_samples = static_cast<double>(results.front().code_phase_samples);
    const auto points = gnss::track(samples, tracking_config, state);
    assert(points.size() == 4);
    assert(points.front().prompt_magnitude > 900.0);
}

void test_pvt() {
    const gnss::Vec3 receiver{6378137.0, 0.0, 0.0};
    constexpr double clock_bias = 75000.0;
    const std::vector<gnss::Vec3> satellites{
        {15600000.0, 0.0, 20180000.0},
        {18760000.0, 13400000.0, 0.0},
        {17610000.0, -13480000.0, 13400000.0},
        {19170000.0, 610000.0, -18390000.0},
        {17800000.0, 9000000.0, 15000000.0},
    };

    std::vector<gnss::SatelliteObservation> observations;
    for (std::size_t i = 0; i < satellites.size(); ++i) {
        observations.push_back(gnss::SatelliteObservation{
            static_cast<int>(i + 1),
            satellites[i],
            gnss::norm(satellites[i] - receiver) + clock_bias,
            1.0,
        });
    }

    const auto solution = gnss::solve_pvt(observations);
    assert(solution.valid);
    assert(gnss::norm(solution.receiver_ecef_m - receiver) < 1e-2);
    assert(std::abs(solution.clock_bias_m - clock_bias) < 1e-2);
    assert(solution.gdop > 0.0);
}

} // namespace

int main() {
    test_ca_code();
    test_acquisition_and_tracking();
    test_pvt();
    return 0;
}
