#include "gnss/math.hpp"
#include "gnss/constants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gnss {
namespace {

constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84B = kWgs84A * (1.0 - kWgs84F);
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);

} // namespace

Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3& a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}

Vec3 operator/(const Vec3& a, double s) {
    return {a.x / s, a.y / s, a.z / s};
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

Geodetic ecef_to_geodetic(const Vec3& ecef_m) {
    const double x = ecef_m.x;
    const double y = ecef_m.y;
    const double z = ecef_m.z;
    const double longitude = std::atan2(y, x);
    const double p = std::hypot(x, y);

    if (p < 1e-9 && std::abs(z) < 1e-9) {
        return {0.0, 0.0, -kWgs84A};
    }

    double latitude = std::atan2(z, p * (1.0 - kWgs84E2));
    double height = 0.0;
    for (int i = 0; i < 10; ++i) {
        const double sin_lat = std::sin(latitude);
        const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
        height = p / std::max(std::cos(latitude), 1e-15) - n;
        latitude = std::atan2(z, p * (1.0 - kWgs84E2 * n / (n + height)));
    }

    if (p < 1e-9) {
        latitude = z >= 0.0 ? kPi / 2.0 : -kPi / 2.0;
        height = std::abs(z) - kWgs84B;
    }

    return {
        latitude * 180.0 / kPi,
        longitude * 180.0 / kPi,
        height,
    };
}

std::array<double, 4> solve_4x4(std::array<std::array<double, 4>, 4> matrix, std::array<double, 4> rhs) {
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        double best = std::abs(matrix[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)]);
        for (int row = col + 1; row < 4; ++row) {
            const double candidate = std::abs(matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]);
            if (candidate > best) {
                best = candidate;
                pivot = row;
            }
        }
        if (best < 1e-18) {
            throw std::runtime_error("Singular 4x4 system");
        }
        if (pivot != col) {
            std::swap(matrix[static_cast<std::size_t>(pivot)], matrix[static_cast<std::size_t>(col)]);
            std::swap(rhs[static_cast<std::size_t>(pivot)], rhs[static_cast<std::size_t>(col)]);
        }

        const double diag = matrix[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)];
        for (int c = col; c < 4; ++c) {
            matrix[static_cast<std::size_t>(col)][static_cast<std::size_t>(c)] /= diag;
        }
        rhs[static_cast<std::size_t>(col)] /= diag;

        for (int row = 0; row < 4; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
            for (int c = col; c < 4; ++c) {
                matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(c)] -=
                    factor * matrix[static_cast<std::size_t>(col)][static_cast<std::size_t>(c)];
            }
            rhs[static_cast<std::size_t>(row)] -= factor * rhs[static_cast<std::size_t>(col)];
        }
    }
    return rhs;
}

std::array<std::array<double, 4>, 4> invert_4x4(const std::array<std::array<double, 4>, 4>& matrix) {
    std::array<std::array<double, 4>, 4> inverse{};
    for (int col = 0; col < 4; ++col) {
        std::array<double, 4> rhs{};
        rhs[static_cast<std::size_t>(col)] = 1.0;
        const auto solution = solve_4x4(matrix, rhs);
        for (int row = 0; row < 4; ++row) {
            inverse[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] = solution[static_cast<std::size_t>(row)];
        }
    }
    return inverse;
}

} // namespace gnss
