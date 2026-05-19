#include "ParticleSystem.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>

template <typename CollisionFn>
double runBenchmark(int n, int steps, float box, float radius, float dt,
                    CollisionFn collisionFn) {
    auto particles = spawnParticles(n, box, radius, 10.0f);
    double simMs = 0.0;
    for (int s = 0; s < steps; ++s) {
        auto t0 = std::chrono::high_resolution_clock::now();
        integrate(particles, dt);
        wallCollide(particles, box, box);
        collisionFn(particles);
        auto t1 = std::chrono::high_resolution_clock::now();
        simMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    return simMs / steps;
}

int main() {
    const float BOX    = 100.0f;
    const float RADIUS = 1.0f;
    const float DT     = 0.1f;
    const int   STEPS  = 60;

    const int Ns[] = { 200, 400, 800, 1600, 3200, 6400, 12800 };

    std::ofstream csv("benchmark.csv");
    csv << "N,naive_ms,hashed_ms\n";

    std::printf("%6s | %12s | %12s | %8s\n",
                "N", "naive ms/f", "hash ms/f", "speedup");
    std::printf("-------+--------------+--------------+----------\n");

    for (int N : Ns) {
        double naiveMs = runBenchmark(N, STEPS, BOX, RADIUS, DT,
            [&](std::vector<Particle>& p) { particleCollide(p, RADIUS); });

        double hashedMs = runBenchmark(N, STEPS, BOX, RADIUS, DT,
            [&](std::vector<Particle>& p) { particleCollideHashed(p, RADIUS, BOX); });

        std::printf("%6d | %12.3f | %12.3f | %7.1fx\n",
                    N, naiveMs, hashedMs, naiveMs / hashedMs);

        csv << N << "," << naiveMs << "," << hashedMs << "\n";
    }

    std::cout << "\nwrote benchmark.csv\n";
    return 0;
}