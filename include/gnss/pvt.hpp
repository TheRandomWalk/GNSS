#pragma once

#include "gnss/math.hpp"

#include <string>
#include <vector>

namespace gnss {

struct SatelliteObservation {
    int prn = 0;
    Vec3 satellite_ecef_m{};
    double pseudorange_m = 0.0;
    double sigma_m = 1.0;
};

struct PVTConfig {
    Vec3 initial_receiver_ecef_m{};
    double initial_clock_bias_m = 0.0;
    int max_iterations = 12;
    double convergence_m = 1e-4;
};

struct PVTSolution {
    bool valid = false;
    Vec3 receiver_ecef_m{};
    Geodetic receiver_lla{};
    double clock_bias_m = 0.0;
    double clock_bias_s = 0.0;
    double rms_residual_m = 0.0;
    double gdop = 0.0;
    double pdop = 0.0;
    double hdop = 0.0;
    double vdop = 0.0;
    double tdop = 0.0;
    int iterations = 0;
    int used_observations = 0;
    std::vector<double> residuals_m;
    std::string message;
};

PVTSolution solve_pvt(const std::vector<SatelliteObservation>& observations, const PVTConfig& config = {});

} // namespace gnss
