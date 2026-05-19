#include "PointCloud.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

// ----- Helper Functs -----

float deg2rad(float deg) {
    return deg * 3.14159265f / 180.0f;
}

Point computeCentroid(const std::vector<Point>& points) {
    double sumX = 0, sumY = 0, sumZ = 0;
    for (const auto& p : points) {
        sumX += p.x;
        sumY += p.y;
        sumZ += p.z;
    }
    size_t n = points.size();
    Point c;
    c.x = static_cast<float>(sumX / n);
    c.y = static_cast<float>(sumY / n);
    c.z = static_cast<float>(sumZ / n);
    return c;
}

// ----- I/O -----

std::vector<Point> loadPly(const std::string& filename) {
    std::vector<Point> points;
    
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "couldnt open " << filename << "\n";
        return points;
    }
    
    size_t vertexCount = 0;
    std::string line;
    
    while (std::getline(file, line)) {
        if (line.rfind("element vertex", 0) == 0) {
            std::istringstream iss(line);
            std::string word1, word2;
            iss >> word1 >> word2 >> vertexCount;
        }
        if (line == "end_header") break;
    }
    
    points.reserve(vertexCount);
    
    for (size_t i = 0; i < vertexCount; ++i) {
        Point p;
        int r, g, b;
        file >> p.x >> p.y >> p.z >> r >> g >> b;
        p.r = static_cast<unsigned char>(r);
        p.g = static_cast<unsigned char>(g);
        p.b = static_cast<unsigned char>(b);
        points.push_back(p);
    }
    
    return points;
}

void savePLY(const std::string& filename, const std::vector<Point>& points) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "couldnt write " << filename << "\n";
        return;
    }
    
    file << "ply\n"
         << "format ascii 1.0\n"
         << "element vertex " << points.size() << "\n"
         << "property float x\n"
         << "property float y\n"
         << "property float z\n"
         << "property uchar red\n"
         << "property uchar green\n"
         << "property uchar blue\n"
         << "end_header\n";
    
    for (const auto& p : points) {
        file << p.x << " " << p.y << " " << p.z << " "
             << (int)p.r << " " << (int)p.g << " " << (int)p.b << "\n";
    }
    
    std::cout << "wrote " << points.size() << " points to " << filename << "\n";
}

// ----- Transformations: -----

void translate(std::vector<Point>& points, float tx, float ty, float tz) {
    for (auto& p : points) {
        p.x += tx;
        p.y += ty;
        p.z += tz;
    }
}

std::array<float, 9> rodriguesMatrix(float ax, float ay, float az, float angleRad) {
    float norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-8f) {
        return {1, 0, 0,  0, 1, 0,  0, 0, 1};
    }
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    float c = std::cos(angleRad);
    float s = std::sin(angleRad);
    float C = 1.0f - c;
    
    return {
        c + ax*ax*C,       ax*ay*C - az*s,    ax*az*C + ay*s,
        ay*ax*C + az*s,    c + ay*ay*C,       ay*az*C - ax*s,
        az*ax*C - ay*s,    az*ay*C + ax*s,    c + az*az*C
    };
}

void applyMatrix(std::vector<Point>& points, const std::array<float, 9>& R) {
    for (auto& p : points) {
        float newX = R[0] * p.x + R[1] * p.y + R[2] * p.z;
        float newY = R[3] * p.x + R[4] * p.y + R[5] * p.z;
        float newZ = R[6] * p.x + R[7] * p.y + R[8] * p.z;
        p.x = newX;
        p.y = newY;
        p.z = newZ;
    }
}

void explode(std::vector<Point>& points, float k) {
    Point c = computeCentroid(points);
    
    for (auto& p : points) {
        float dx = p.x - c.x;
        float dy = p.y - c.y;
        float dz = p.z - c.z;
        
        float len = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (len < 1e-8f) continue;
        
        float scale = k / len;
        p.x += dx * scale;
        p.y += dy * scale;
        p.z += dz * scale;
    }
}

// ----- SoA conversion -----

PointCloudSoA toSoA(const std::vector<Point>& aos) {
    PointCloudSoA soa;
    const size_t n = aos.size();
    
    // Pre-allocate all six vectors
    soa.xs.resize(n);
    soa.ys.resize(n);
    soa.zs.resize(n);
    soa.rs.resize(n);
    soa.gs.resize(n);
    soa.bs.resize(n);
    
    // Single pass: read each Point, scatter components into parallel arrays
    for (size_t i = 0; i < n; ++i) {
        soa.xs[i] = aos[i].x;
        soa.ys[i] = aos[i].y;
        soa.zs[i] = aos[i].z;
        soa.rs[i] = aos[i].r;
        soa.gs[i] = aos[i].g;
        soa.bs[i] = aos[i].b;
    }
    return soa;
}

std::vector<Point> toAoS(const PointCloudSoA& soa) {
    std::vector<Point> aos;
    const size_t n = soa.size();
    aos.resize(n);
    
    for (size_t i = 0; i < n; ++i) {
        aos[i].x = soa.xs[i];
        aos[i].y = soa.ys[i];
        aos[i].z = soa.zs[i];
        aos[i].r = soa.rs[i];
        aos[i].g = soa.gs[i];
        aos[i].b = soa.bs[i];
    }
    return aos;
}

// ----- SoA versions of transformations -----

void translateSoA(PointCloudSoA& cloud, float tx, float ty, float tz) {
    const size_t n = cloud.size();
    for (size_t i = 0; i < n; ++i) {
        cloud.xs[i] += tx;
        cloud.ys[i] += ty;
        cloud.zs[i] += tz;
    }
}

void applyMatrixSoA(PointCloudSoA& cloud, const std::array<float, 9>& R) {
    const float r00 = R[0], r01 = R[1], r02 = R[2];
    const float r10 = R[3], r11 = R[4], r12 = R[5];
    const float r20 = R[6], r21 = R[7], r22 = R[8];
    
    const size_t n = cloud.size();
    for (size_t i = 0; i < n; ++i) {
        float x = cloud.xs[i];
        float y = cloud.ys[i];
        float z = cloud.zs[i];
        cloud.xs[i] = r00*x + r01*y + r02*z;
        cloud.ys[i] = r10*x + r11*y + r12*z;
        cloud.zs[i] = r20*x + r21*y + r22*z;
    }
}

void explodeSoA(PointCloudSoA& cloud, float k) {
    const size_t n = cloud.size();
    if (n == 0) return;
    
    // Centroid in one pass
    double sumX = 0, sumY = 0, sumZ = 0;
    for (size_t i = 0; i < n; ++i) {
        sumX += cloud.xs[i];
        sumY += cloud.ys[i];
        sumZ += cloud.zs[i];
    }
    const float cx = static_cast<float>(sumX / n);
    const float cy = static_cast<float>(sumY / n);
    const float cz = static_cast<float>(sumZ / n);
    
    // Push outward
    for (size_t i = 0; i < n; ++i) {
        const float dx = cloud.xs[i] - cx;
        const float dy = cloud.ys[i] - cy;
        const float dz = cloud.zs[i] - cz;
        const float len = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (len < 1e-8f) continue;
        const float scale = k / len;
        cloud.xs[i] += dx * scale;
        cloud.ys[i] += dy * scale;
        cloud.zs[i] += dz * scale;
    }
}