#pragma once

#include <complex>
#include <vector>

namespace gnss {

struct TrackingConfig {
    double sample_rate_hz = 2.046e6;
    double integration_ms = 1.0;
    double early_late_spacing_chips = 0.5;
    double dll_gain = 0.05;
    int max_epochs = 100;
};

struct TrackingState {
    int prn = 1;
    double code_phase_samples = 0.0;
    double doppler_hz = 0.0;
    double carrier_phase_rad = 0.0;
};

struct TrackingPoint {
    int epoch = 0;
    double code_phase_samples = 0.0;
    double doppler_hz = 0.0;
    double prompt_i = 0.0;
    double prompt_q = 0.0;
    double early_magnitude = 0.0;
    double prompt_magnitude = 0.0;
    double late_magnitude = 0.0;
    double dll_discriminator = 0.0;
    double cn0_estimate_dbhz = 0.0;
};

std::vector<TrackingPoint> track(
    const std::vector<std::complex<float>>& samples,
    const TrackingConfig& config,
    TrackingState initial_state);

} // namespace gnss
