#pragma once

#include <array>
#include <vector>

namespace gnss {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Geodetic {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double height_m = 0.0;
};

Vec3 operator+(const Vec3& a, const Vec3& b);
Vec3 operator-(const Vec3& a, const Vec3& b);
Vec3 operator*(const Vec3& a, double s);
Vec3 operator/(const Vec3& a, double s);
double dot(const Vec3& a, const Vec3& b);
double norm(const Vec3& a);
Geodetic ecef_to_geodetic(const Vec3& ecef_m);
std::array<double, 4> solve_4x4(std::array<std::array<double, 4>, 4> matrix, std::array<double, 4> rhs);
std::array<std::array<double, 4>, 4> invert_4x4(const std::array<std::array<double, 4>, 4>& matrix);

} // namespace gnss
