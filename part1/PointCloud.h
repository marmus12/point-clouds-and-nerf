#pragma once

#include <vector>
#include <string>
#include <array>

// Representation of a single point:
struct Point {
    float x, y, z;
    unsigned char r, g, b;
};
// Existing Array-of-Structures layout. Just an alias for clarity in
// the benchmark code below.
using PointCloudAoS = std::vector<Point>;

// Structure of Arrays representation: parallel vectors for each component.
// Faster than AoS for hot loops that only touch xyz, because:
//   - RGB doesn't get pulled into cache
//   - Float arrays are SIMD-friendly (16 floats/cache line)
struct PointCloudSoA {
    std::vector<float> xs, ys, zs;
    std::vector<unsigned char> rs, gs, bs;
    
    size_t size() const { return xs.size(); }
};
// ----- SoA conversion -----

// Convert AoS to SoA (for comparison / benchmarking).
PointCloudSoA toSoA(const std::vector<Point>& aos);

// Convert SoA back to AoS (for saving via existing savePLY).
std::vector<Point> toAoS(const PointCloudSoA& soa);

// ----- I/O -----
std::vector<Point> loadPly(const std::string& filename);
void savePLY(const std::string& filename, const std::vector<Point>& points);

// ----- Transformations -----
void translate(std::vector<Point>& points, float tx, float ty, float tz);

std::array<float, 9> rodriguesMatrix(float ax, float ay, float az, float angleRad);
void applyMatrix(std::vector<Point>& points, const std::array<float, 9>& R);

void explode(std::vector<Point>& points, float k);

// ----- SoA versions of transformations -----
// These run directly on parallel arrays; faster for hot loops.

void translateSoA(PointCloudSoA& cloud, float tx, float ty, float tz);

void applyMatrixSoA(PointCloudSoA& cloud, const std::array<float, 9>& R);

void explodeSoA(PointCloudSoA& cloud, float k);
// ----- Helpers -----
float deg2rad(float deg);
Point computeCentroid(const std::vector<Point>& points);