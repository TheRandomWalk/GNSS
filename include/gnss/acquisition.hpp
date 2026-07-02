#pragma once

#include <complex>
#include <vector>

namespace gnss {

struct AcquisitionConfig {
    double sample_rate_hz = 2.046e6;
    double doppler_min_hz = -5000.0;
    double doppler_max_hz = 5000.0;
    double doppler_step_hz = 500.0;
    double threshold = 2.5;
};

struct AcquisitionResult {
    int prn = 0;
    bool detected = false;
    double doppler_hz = 0.0;
    int code_phase_samples = 0;
    double metric = 0.0;
    double cn0_estimate_dbhz = 0.0;
};

std::vector<AcquisitionResult> acquire(
    const std::vector<std::complex<float>>& samples,
    const AcquisitionConfig& config,
    int first_prn,
    int last_prn);

} // namespace gnss
