#include "gnss/tracking.hpp"
#include "gnss/ca_code.hpp"
#include "gnss/constants.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>

namespace gnss {
namespace {

int positive_mod(int value, int modulus) {
    const int result = value % modulus;
    return result < 0 ? result + modulus : result;
}

double magnitude(const std::complex<double>& value) {
    return std::abs(value);
}

std::complex<double> correlate_epoch(
    const std::vector<std::complex<float>>& samples,
    std::size_t start,
    int count,
    double sample_rate_hz,
    const std::vector<int>& code,
    double code_phase_chips,
    double chip_offset,
    double doppler_hz) {
    std::complex<double> acc{0.0, 0.0};
    const double two_pi = 2.0 * std::numbers::pi;
    for (int i = 0; i < count; ++i) {
        const std::size_t sample_index = start + static_cast<std::size_t>(i);
        const double t = static_cast<double>(sample_index) / sample_rate_hz;
        const double carrier_phase = -two_pi * doppler_hz * t;
        const std::complex<double> carrier{std::cos(carrier_phase), std::sin(carrier_phase)};
        const double chip_phase = code_phase_chips + chip_offset + static_cast<double>(i) * kGpsCaCodeRateHz / sample_rate_hz;
        const int chip_index = positive_mod(static_cast<int>(std::floor(chip_phase)), kGpsCaCodeLength);
        const double code_value = static_cast<double>(code[static_cast<std::size_t>(chip_index)]);
        acc += static_cast<std::complex<double>>(samples[sample_index]) * carrier * code_value;
    }
    return acc;
}

} // namespace

std::vector<TrackingPoint> track(
    const std::vector<std::complex<float>>& samples,
    const TrackingConfig& config,
    TrackingState initial_state) {
    std::vector<TrackingPoint> points;
    if (config.sample_rate_hz <= 0.0 || config.integration_ms <= 0.0 || config.max_epochs <= 0) {
        return points;
    }

    const int samples_per_epoch = static_cast<int>(std::llround(config.sample_rate_hz * config.integration_ms / 1000.0));
    if (samples_per_epoch <= 0 || samples.size() < static_cast<std::size_t>(samples_per_epoch)) {
        return points;
    }

    const auto code = generate_ca_code(initial_state.prn);
    const int epochs = std::min<int>(config.max_epochs, static_cast<int>(samples.size() / static_cast<std::size_t>(samples_per_epoch)));
    points.reserve(static_cast<std::size_t>(epochs));

    double code_phase_chips = initial_state.code_phase_samples * kGpsCaCodeRateHz / config.sample_rate_hz;
    double doppler_hz = initial_state.doppler_hz;
    const double epoch_chips = kGpsCaCodeRateHz * config.integration_ms / 1000.0;

    for (int epoch = 0; epoch < epochs; ++epoch) {
        const std::size_t start = static_cast<std::size_t>(epoch * samples_per_epoch);
        const auto early = correlate_epoch(samples, start, samples_per_epoch, config.sample_rate_hz, code,
                                           code_phase_chips, -config.early_late_spacing_chips, doppler_hz);
        const auto prompt = correlate_epoch(samples, start, samples_per_epoch, config.sample_rate_hz, code,
                                            code_phase_chips, 0.0, doppler_hz);
        const auto late = correlate_epoch(samples, start, samples_per_epoch, config.sample_rate_hz, code,
                                          code_phase_chips, config.early_late_spacing_chips, doppler_hz);

        const double e_mag = magnitude(early);
        const double p_mag = magnitude(prompt);
        const double l_mag = magnitude(late);
        const double discriminator = (e_mag + l_mag) <= 1e-12 ? 0.0 : (e_mag - l_mag) / (e_mag + l_mag);
        const double cn0 = 10.0 * std::log10(std::max((p_mag * p_mag) / static_cast<double>(samples_per_epoch), 1e-20) * 1000.0);

        points.push_back(TrackingPoint{
            epoch,
            code_phase_chips * config.sample_rate_hz / kGpsCaCodeRateHz,
            doppler_hz,
            prompt.real(),
            prompt.imag(),
            e_mag,
            p_mag,
            l_mag,
            discriminator,
            cn0,
        });

        code_phase_chips += epoch_chips + config.dll_gain * discriminator;
        code_phase_chips = std::fmod(code_phase_chips, static_cast<double>(kGpsCaCodeLength));
        if (code_phase_chips < 0.0) {
            code_phase_chips += kGpsCaCodeLength;
        }
    }

    return points;
}

} // namespace gnss
