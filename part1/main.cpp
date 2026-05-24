#include "PointCloud.h"

#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>

using clk = std::chrono::high_resolution_clock;

template <typename F>
double timeMs(F&& f) {
    auto t0 = clk::now();
    f();
    auto t1 = clk::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

int main() {
    // ----- Load -----
    auto loadStart = clk::now();
    std::vector<Point> original = loadPly("Loot.ply");
    auto loadEnd = clk::now();
    double loadMs = std::chrono::duration<double, std::milli>(loadEnd - loadStart).count();

    if (original.empty()) return 1;

    const size_t N = original.size();
    std::cout << "loaded " << N << " points in " << loadMs << " ms\n\n";

    // ----- Convert once to SoA -----
    PointCloudSoA originalSoA;
    double conversionMs = timeMs([&] {
        originalSoA = toSoA(original);
    });
    std::cout << "AoS -> SoA conversion: " << conversionMs << " ms\n\n";

    // ----- Benchmark table header -----
    std::printf("%-22s | %10s | %10s | %8s\n",
                "operation", "AoS ms", "SoA ms", "speedup");
    std::printf("-----------------------+------------+------------+---------\n");

    const float OFFSET = 800.0f;
    const int   RUNS   = 10;

    // --- 1. Translation ---
    {
        std::vector<double> aosTimes, soaTimes;
        for (int i = 0; i < RUNS; ++i) {
            auto aos = original;
            auto soa = originalSoA;
            aosTimes.push_back(timeMs([&] { translate(aos, 0.0f, 200.0f, 0.0f); }));
            soaTimes.push_back(timeMs([&] { translateSoA(soa, 0.0f, 200.0f, 0.0f); }));
        }
        std::sort(aosTimes.begin(), aosTimes.end());
        std::sort(soaTimes.begin(), soaTimes.end());
        double aosMs = aosTimes[RUNS / 2];
        double soaMs = soaTimes[RUNS / 2];
        std::printf("%-22s | %10.3f | %10.3f | %7.2fx\n",
                    "translate", aosMs, soaMs, aosMs / soaMs);

        auto aos = original;
        translate(aos, 0.0f, 200.0f, 0.0f);
        translate(aos, OFFSET, 0.0f, 0.0f);
        savePLY("out_1_translated.ply", aos);
    }

    // --- 2. Rotation around Y axis ---
    {
        auto R = rodriguesMatrix(0.0f, 1.0f, 0.0f, deg2rad(90.0f));
        std::vector<double> aosTimes, soaTimes;
        for (int i = 0; i < RUNS; ++i) {
            auto aos = original;
            auto soa = originalSoA;
            aosTimes.push_back(timeMs([&] { applyMatrix(aos, R); }));
            soaTimes.push_back(timeMs([&] { applyMatrixSoA(soa, R); }));
        }
        std::sort(aosTimes.begin(), aosTimes.end());
        std::sort(soaTimes.begin(), soaTimes.end());
        double aosMs = aosTimes[RUNS / 2];
        double soaMs = soaTimes[RUNS / 2];
        std::printf("%-22s | %10.3f | %10.3f | %7.2fx\n",
                    "rotate Y (90deg)", aosMs, soaMs, aosMs / soaMs);

        auto aos = original;
        applyMatrix(aos, R);
        translate(aos, 2 * OFFSET, 0.0f, 0.0f);
        savePLY("out_2_rotated_y.ply", aos);
    }

    // --- 3. Tilted rotation ---
    {
        auto R = rodriguesMatrix(1.0f, 1.0f, 0.0f, deg2rad(45.0f));
        std::vector<double> aosTimes, soaTimes;
        for (int i = 0; i < RUNS; ++i) {
            auto aos = original;
            auto soa = originalSoA;
            aosTimes.push_back(timeMs([&] { applyMatrix(aos, R); }));
            soaTimes.push_back(timeMs([&] { applyMatrixSoA(soa, R); }));
        }
        std::sort(aosTimes.begin(), aosTimes.end());
        std::sort(soaTimes.begin(), soaTimes.end());
        double aosMs = aosTimes[RUNS / 2];
        double soaMs = soaTimes[RUNS / 2];
        std::printf("%-22s | %10.3f | %10.3f | %7.2fx\n",
                    "rotate tilted (45)", aosMs, soaMs, aosMs / soaMs);

        auto aos = original;
        applyMatrix(aos, R);
        translate(aos, 3 * OFFSET, 0.0f, 0.0f);
        savePLY("out_3_rotated_tilted.ply", aos);
    }

    // --- 4. Explode ---
    {
        std::vector<double> aosTimes, soaTimes;
        for (int i = 0; i < RUNS; ++i) {
            auto aos = original;
            auto soa = originalSoA;
            aosTimes.push_back(timeMs([&] { explode(aos, 100.0f); }));
            soaTimes.push_back(timeMs([&] { explodeSoA(soa, 100.0f); }));
        }
        std::sort(aosTimes.begin(), aosTimes.end());
        std::sort(soaTimes.begin(), soaTimes.end());
        double aosMs = aosTimes[RUNS / 2];
        double soaMs = soaTimes[RUNS / 2];
        std::printf("%-22s | %10.3f | %10.3f | %7.2fx\n",
                    "explode displace", aosMs, soaMs, aosMs / soaMs);

        auto aos = original;
        explode(aos, 100.0f);
        translate(aos, 4 * OFFSET, 0.0f, 0.0f);
        savePLY("out_4_exploded.ply", aos);
    }

    std::cout << "\nAll 4 transformations saved as out_*.ply\n";
    return 0;
}