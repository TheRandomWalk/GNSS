#include "gnss/pvt.hpp"
#include "gnss/constants.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace gnss {
namespace {

using Matrix4 = std::array<std::array<double, 4>, 4>;
using Vector4 = std::array<double, 4>;

void add_normal_row(Matrix4& normal, Vector4& rhs, const Vector4& h, double residual, double weight) {
    for (int r = 0; r < 4; ++r) {
        rhs[static_cast<std::size_t>(r)] += h[static_cast<std::size_t>(r)] * residual * weight;
        for (int c = 0; c < 4; ++c) {
            normal[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] +=
                h[static_cast<std::size_t>(r)] * h[static_cast<std::size_t>(c)] * weight;
        }
    }
}

std::array<double, 3> enu_variances_from_ecef_cov(const Matrix4& q, const Geodetic& lla) {
    const double lat = lla.latitude_deg * kPi / 180.0;
    const double lon = lla.longitude_deg * kPi / 180.0;
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    const std::array<std::array<double, 3>, 3> r{{
        {{-sin_lon, cos_lon, 0.0}},
        {{-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat}},
        {{cos_lat * cos_lon, cos_lat * sin_lon, sin_lat}},
    }};

    std::array<double, 3> variance{};
    for (int axis = 0; axis < 3; ++axis) {
        double v = 0.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                v += r[static_cast<std::size_t>(axis)][static_cast<std::size_t>(i)] *
                     q[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                     r[static_cast<std::size_t>(axis)][static_cast<std::size_t>(j)];
            }
        }
        variance[static_cast<std::size_t>(axis)] = std::max(0.0, v);
    }
    return variance;
}

} // namespace

PVTSolution solve_pvt(const std::vector<SatelliteObservation>& observations, const PVTConfig& config) {
    PVTSolution solution;
    solution.used_observations = static_cast<int>(observations.size());
    if (observations.size() < 4) {
        solution.message = "At least 4 satellite observations are required for PVT";
        return solution;
    }

    Vec3 receiver = config.initial_receiver_ecef_m;
    double clock_bias = config.initial_clock_bias_m;
    std::vector<double> residuals(observations.size());

    try {
        for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
            Matrix4 normal{};
            Vector4 rhs{};

            for (std::size_t i = 0; i < observations.size(); ++i) {
                const auto& observation = observations[i];
                const Vec3 los_vector = observation.satellite_ecef_m - receiver;
                const double range = std::max(norm(los_vector), 1.0);
                const Vec3 unit = los_vector / range;
                const double predicted = range + clock_bias;
                const double residual = observation.pseudorange_m - predicted;
                const double sigma = std::max(observation.sigma_m, 0.01);
                const double weight = 1.0 / (sigma * sigma);
                const Vector4 h{{-unit.x, -unit.y, -unit.z, 1.0}};
                residuals[i] = residual;
                add_normal_row(normal, rhs, h, residual, weight);
            }

            const auto update = solve_4x4(normal, rhs);
            receiver.x += update[0];
            receiver.y += update[1];
            receiver.z += update[2];
            clock_bias += update[3];
            solution.iterations = iteration + 1;

            const double step = std::sqrt(update[0] * update[0] + update[1] * update[1] + update[2] * update[2]);
            if (step < config.convergence_m && std::abs(update[3]) < config.convergence_m) {
                break;
            }
        }

        double rss = 0.0;
        for (std::size_t i = 0; i < observations.size(); ++i) {
            const auto& observation = observations[i];
            const double range = norm(observation.satellite_ecef_m - receiver);
            residuals[i] = observation.pseudorange_m - (range + clock_bias);
            rss += residuals[i] * residuals[i];
        }

        Matrix4 unweighted_normal{};
        Vector4 dummy{};
        for (const auto& observation : observations) {
            const Vec3 los_vector = observation.satellite_ecef_m - receiver;
            const double range = std::max(norm(los_vector), 1.0);
            const Vec3 unit = los_vector / range;
            const Vector4 h{{-unit.x, -unit.y, -unit.z, 1.0}};
            add_normal_row(unweighted_normal, dummy, h, 0.0, 1.0);
        }
        const auto q = invert_4x4(unweighted_normal);
        const auto lla = ecef_to_geodetic(receiver);
        const auto enu_variance = enu_variances_from_ecef_cov(q, lla);

        solution.valid = true;
        solution.receiver_ecef_m = receiver;
        solution.receiver_lla = lla;
        solution.clock_bias_m = clock_bias;
        solution.clock_bias_s = clock_bias / kSpeedOfLightMps;
        solution.residuals_m = residuals;
        solution.rms_residual_m = std::sqrt(rss / std::max<std::size_t>(1, observations.size() - 4));
        solution.pdop = std::sqrt(std::max(0.0, q[0][0] + q[1][1] + q[2][2]));
        solution.tdop = std::sqrt(std::max(0.0, q[3][3]));
        solution.gdop = std::sqrt(std::max(0.0, solution.pdop * solution.pdop + solution.tdop * solution.tdop));
        solution.hdop = std::sqrt(enu_variance[0] + enu_variance[1]);
        solution.vdop = std::sqrt(enu_variance[2]);
        solution.message = "PVT solution converged";
    } catch (const std::exception& error) {
        solution.valid = false;
        solution.message = error.what();
    }

    return solution;
}

} // namespace gnss
