#include "ParticleSystem.h"
#include <iostream>
#include <cmath>
#include <random>

// ----- Core simulation steps -----

void integrate(std::vector<Particle>& particles, float dt) {
    for (auto& p : particles) {
        p.x += p.vx * dt;
        p.y += p.vy * dt;
    }
}

void wallCollide(std::vector<Particle>& particles, float sizeX, float sizeY) {
    for (auto& p : particles) {
        if (p.x < 0.0f)   { p.x = 0.0f;   p.vx = -p.vx; }
        if (p.x > sizeX)  { p.x = sizeX;  p.vx = -p.vx; }
        if (p.y < 0.0f)   { p.y = 0.0f;   p.vy = -p.vy; }
        if (p.y > sizeY)  { p.y = sizeY;  p.vy = -p.vy; }
    }
}

void particleCollide(std::vector<Particle>& particles, float radius) {
    const float minDist   = 2.0f * radius;
    const float minDistSq = minDist * minDist;
    const size_t n = particles.size();

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            Particle& a = particles[i];
            Particle& b = particles[j];

            float dx = b.x - a.x;
            float dy = b.y - a.y;
            float distSq = dx*dx + dy*dy;

            if (distSq >= minDistSq || distSq < 1e-12f) continue;

            float dist = std::sqrt(distSq);
            float nx = dx / dist;
            float ny = dy / dist;

            // Positional correction 
            float overlap = (minDist - dist) * 0.5f;
            a.x -= nx * overlap; a.y -= ny * overlap;
            b.x += nx * overlap; b.y += ny * overlap;

            // Velocity response
            float vaN = a.vx*nx + a.vy*ny;
            float vbN = b.vx*nx + b.vy*ny;
            if (vaN - vbN <= 0.0f) continue;
            float dV = vbN - vaN;
            a.vx += dV*nx; a.vy += dV*ny;
            b.vx -= dV*nx; b.vy -= dV*ny;
        }
    }
}

void particleCollideHashed(std::vector<Particle>& particles,
                           float radius, float boxSize) {
    const float cellSize  = 2.0f * radius;
    const int   gridDim   = static_cast<int>(std::ceil(boxSize / cellSize));
    const float minDist   = 2.0f * radius;
    const float minDistSq = minDist * minDist;

    // Build grid
    std::vector<std::vector<int>> cells(gridDim * gridDim);

    auto cellIdx = [&](float x, float y) {
        int cx = static_cast<int>(x / cellSize);
        int cy = static_cast<int>(y / cellSize);
        if (cx < 0) cx = 0; else if (cx >= gridDim) cx = gridDim - 1;
        if (cy < 0) cy = 0; else if (cy >= gridDim) cy = gridDim - 1;
        return cy * gridDim + cx;
    };

    for (size_t i = 0; i < particles.size(); ++i)
        cells[cellIdx(particles[i].x, particles[i].y)].push_back(static_cast<int>(i));

    // Resolve collisions
    static const int dxs[4] = { +1, +1,  0, -1 };
    static const int dys[4] = {  0, +1, +1, +1 };

    auto resolvePair = [&](int i, int j) {
        Particle& a = particles[i];
        Particle& b = particles[j];
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float distSq = dx*dx + dy*dy;
        if (distSq >= minDistSq || distSq < 1e-12f) return;
        float dist = std::sqrt(distSq);
        float nx = dx/dist, ny = dy/dist;
        float overlap = (minDist - dist) * 0.5f;
        a.x -= nx*overlap; a.y -= ny*overlap;
        b.x += nx*overlap; b.y += ny*overlap;
        float vaN = a.vx*nx + a.vy*ny;
        float vbN = b.vx*nx + b.vy*ny;
        if (vaN - vbN <= 0.0f) return;
        float dV = vbN - vaN;
        a.vx += dV*nx; a.vy += dV*ny;
        b.vx -= dV*nx; b.vy -= dV*ny;
    };

    for (int cy = 0; cy < gridDim; ++cy) {
        for (int cx = 0; cx < gridDim; ++cx) {
            int c = cy * gridDim + cx;
            const auto& cell = cells[c];

            // Same cell
            for (size_t i = 0; i < cell.size(); ++i)
                for (size_t j = i + 1; j < cell.size(); ++j)
                    resolvePair(cell[i], cell[j]);

            // 4 forward neighbors
            for (int k = 0; k < 4; ++k) {
                int nnx = cx + dxs[k];
                int nny = cy + dys[k];
                if (nnx < 0 || nnx >= gridDim ||
                    nny < 0 || nny >= gridDim) continue;
                const auto& nb = cells[nny * gridDim + nnx];
                for (int i : cell)
                    for (int j : nb)
                        resolvePair(i, j);
            }
        }
    }
}

// ----- Utilities -----

std::vector<Particle> spawnParticles(int n, float box,
                                     float radius, float maxSpeed,
                                     unsigned int seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> pos(radius, box - radius);
    std::uniform_real_distribution<float> vel(-maxSpeed, maxSpeed);

    std::vector<Particle> particles;
    particles.reserve(n);

    // Rejection sampling: reject any new position that overlaps with
    // an already-placed particle. This is O(N^2) but only runs once
    // at initialization, so it doesn't affect the per-frame cost.
    const float minDistSq = 4.0f * radius * radius;
    const int   maxAttempts = 10000;
    int attempts = 0;

    while ((int)particles.size() < n) {
        Particle p { pos(rng), pos(rng), vel(rng), vel(rng) };
        bool ok = true;
        for (const auto& q : particles) {
            float dx = q.x - p.x;
            float dy = q.y - p.y;
            if (dx * dx + dy * dy < minDistSq) { ok = false; break; }
        }
        if (ok) particles.push_back(p);
        if (++attempts > maxAttempts) {
            std::cerr << "spawnParticles: failed after " << maxAttempts
                      << " attempts. Density too high?\n";
            break;
        }
    }
    return particles;
}