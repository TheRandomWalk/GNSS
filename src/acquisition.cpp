#include "gnss/acquisition.hpp"
#include "gnss/ca_code.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <numeric>

namespace gnss {
namespace {

struct Peak {
    double value = 0.0;
    double doppler = 0.0;
    int phase = 0;
};

double average_power(const std::vector<std::complex<float>>& samples, int count) {
    double sum = 0.0;
    for (int i = 0; i < count; ++i) {
        sum += std::norm(samples[static_cast<std::size_t>(i)]);
    }
    return count == 0 ? 0.0 : sum / static_cast<double>(count);
}

} // namespace

std::vector<AcquisitionResult> acquire(
    const std::vector<std::complex<float>>& samples,
    const AcquisitionConfig& config,
    int first_prn,
    int last_prn) {
    const int samples_per_ms = static_cast<int>(std::llround(config.sample_rate_hz / 1000.0));
    const int coherent_samples = std::min(samples_per_ms, static_cast<int>(samples.size()));
    std::vector<AcquisitionResult> results;
    if (coherent_samples < 1023) {
        return results;
    }

    const double input_power = std::max(average_power(samples, coherent_samples), 1e-20);
    const double two_pi = 2.0 * std::numbers::pi;

    for (int prn = first_prn; prn <= last_prn; ++prn) {
        const auto code = sampled_ca_code(prn, config.sample_rate_hz, coherent_samples);
        Peak best;
        double sum = 0.0;
        int bins = 0;

        for (double doppler = config.doppler_min_hz; doppler <= config.doppler_max_hz + 0.1; doppler += config.doppler_step_hz) {
            std::vector<std::complex<float>> wiped(static_cast<std::size_t>(coherent_samples));
            for (int i = 0; i < coherent_samples; ++i) {
                const double phase = -two_pi * doppler * static_cast<double>(i) / config.sample_rate_hz;
                const std::complex<float> carrier(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
                wiped[static_cast<std::size_t>(i)] = samples[static_cast<std::size_t>(i)] * carrier;
            }

            for (int code_phase = 0; code_phase < coherent_samples; ++code_phase) {
                std::complex<double> acc{0.0, 0.0};
                for (int i = 0; i < coherent_samples; ++i) {
                    const int code_index = (i + code_phase) % coherent_samples;
                    acc += static_cast<double>(code[static_cast<std::size_t>(code_index)]) *
                           static_cast<std::complex<double>>(wiped[static_cast<std::size_t>(i)]);
                }
                const double value = std::norm(acc);
                best = value > best.value ? Peak{value, doppler, code_phase} : best;
                sum += value;
                ++bins;
            }
        }

        const double mean = bins == 0 ? 0.0 : sum / static_cast<double>(bins);
        const double metric = best.value / std::max(mean, 1e-20);
        const double cn0 = 10.0 * std::log10(std::max(best.value / (input_power * coherent_samples), 1e-20) * 1000.0);
        results.push_back(AcquisitionResult{
            prn,
            metric >= config.threshold,
            best.doppler,
            best.phase,
            metric,
            cn0,
        });
    }

    std::sort(results.begin(), results.end(), [](const auto& a, const auto& b) {
        if (a.detected != b.detected) {
            return a.detected > b.detected;
        }
        return a.metric > b.metric;
    });
    return results;
}

} // namespace gnss
