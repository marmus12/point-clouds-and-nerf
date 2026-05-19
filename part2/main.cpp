#include "ParticleSystem.h"

#include <iostream>
#include <fstream>
#include <chrono>

int main() {
    const int   N         = 800;
    const float BOX       = 100.0f;
    const float RADIUS    = 1.0f;
    const float DT        = 0.1f;
    const int   STEPS     = 600;
    const float MAX_SPEED = 10.0f;

    auto particles = spawnParticles(N, BOX, RADIUS, MAX_SPEED);
    std::cout << "spawned " << particles.size() << " particles.\n";

    std::ofstream out("frames.csv");
    out << "step,id,x,y\n";

    double simMs = 0.0, writeMs = 0.0;

    for (int s = 0; s <= STEPS; ++s) {
        // Simulation step
        auto t0 = std::chrono::high_resolution_clock::now();
        if (s > 0) {
            integrate(particles, DT);
            wallCollide(particles, BOX, BOX);
            particleCollideHashed(particles, RADIUS, BOX);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        simMs += std::chrono::duration<double, std::milli>(t1 - t0).count();

        // Write frame
        auto t2 = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < particles.size(); ++i)
            out << s << "," << i << ","
                << particles[i].x << "," << particles[i].y << "\n";
        auto t3 = std::chrono::high_resolution_clock::now();
        writeMs += std::chrono::duration<double, std::milli>(t3 - t2).count();
    }

    std::cout << "simulation:  " << simMs
              << " ms  (" << (simMs / STEPS) << " ms/frame)\n";
    std::cout << "csv writing: " << writeMs << " ms\n";
    std::cout << "fps equiv (sim only): "
              << (1000.0 * STEPS / simMs) << "\n";

    return 0;
}