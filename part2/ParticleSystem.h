#pragma once

#include <vector>

struct Particle {
    float x, y;     // position
    float vx, vy;   // velocity
};

// ----- Core simulation steps -----
void integrate(std::vector<Particle>& particles, float dt);

void wallCollide(std::vector<Particle>& particles, float sizeX, float sizeY);

// Naive O(N^2) collision detection.
void particleCollide(std::vector<Particle>& particles, float radius);

// Spatial-hash collision detection, O(N) expected.
void particleCollideHashed(std::vector<Particle>& particles,
                           float radius, float boxSize);

// ----- Utilities -----

// Spawn N particles randomly inside the box.
std::vector<Particle> spawnParticles(int n, float box,
                                     float radius, float maxSpeed,
                                     unsigned int seed = 1234);