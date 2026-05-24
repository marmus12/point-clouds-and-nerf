#include "PointCloud.h"

#include <iostream>
#include <chrono>

using clk = std::chrono::high_resolution_clock;

template <typename F>
double timeMs(F&& f) {
    auto t0 = clk::now();
    f();
    auto t1 = clk::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

int main() {
    auto loadStart = clk::now();
    std::vector<Point> original = loadPly("Loot.ply");
    auto loadEnd = clk::now();
    double loadMs = std::chrono::duration<double, std::milli>(loadEnd - loadStart).count();
    
    if (original.empty()) return 1;
    
    const size_t N = original.size();
    std::cout << "loaded " << N << " points in " 
              << loadMs << " ms\n\n";
    
    // ----- Convert once to SoA for the SoA benchmark -----

    PointCloudSoA originalSoA;
    double conversionMs = timeMs([&] {
        originalSoA = toSoA(original);
    });
    std::cout << "AoS -> SoA conversion: " << conversionMs << " ms\n\n";
    
    // ----- Header for benchmark table -----
    std::printf("%-22s | %10s | %10s | %8s\n",
                "operation", "AoS ms", "SoA ms", "speedup");
    std::printf("-----------------------+------------+------------+---------\n");
    
    const float OFFSET = 800.0f;
    
    // --- 1. Translation ---
    {
        auto aos = original;
        auto soa = originalSoA;
        double aosMs = timeMs([&] { translate(aos, 0.0f, 200.0f, 0.0f); });
        double soaMs = timeMs([&] { translateSoA(soa, 0.0f, 200.0f, 0.0f); });
        std::printf("%-22s | %10.3f | %10.3f | %7.2fx\n",
                    "translate", aosMs, soaMs, aosMs / soaMs);
        
        translate(aos, OFFSET, 0.0f, 0.0f);   // visualization offset
        savePLY("out_1_translated.ply", aos);
    }
    
    // --- 2. Rotation around Y axis ---
    {
        auto aos = original;
        auto soa = originalSoA;
        auto R = rodriguesMatrix(0.0f, 1.0f, 0.0f, deg2rad(90.0f));
        double aosMs = timeMs([&] { applyMatrix(aos, R); });
        double soaMs = timeMs([&] { applyMatrixSoA(soa, R); });
        std::printf("%-22s | %10.3f | %10.3f | %7.2fx\n",
                    "rotate Y (90deg)", aosMs, soaMs, aosMs / soaMs);
        
        translate(aos, 2 * OFFSET, 0.0f, 0.0f);
        savePLY("out_2_rotated_y.ply", aos);
    }
    
    // --- 3. Tilted rotation ---
    {
        auto aos = original;
        auto soa = originalSoA;
        auto R = rodriguesMatrix(1.0f, 1.0f, 0.0f, deg2rad(45.0f));
        double aosMs = timeMs([&] { applyMatrix(aos, R); });
        double soaMs = timeMs([&] { applyMatrixSoA(soa, R); });
        std::printf("%-22s | %10.3f | %10.3f | %7.2fx\n",
                    "rotate tilted (45)", aosMs, soaMs, aosMs / soaMs);
        
        translate(aos, 3 * OFFSET, 0.0f, 0.0f);
        savePLY("out_3_rotated_tilted.ply", aos);
    }
    
    // --- 4. Explode ---
    {
        auto aos = original;
        auto soa = originalSoA;
        double aosMs = timeMs([&] { explode(aos, 100.0f); });
        double soaMs = timeMs([&] { explodeSoA(soa, 100.0f); });
        std::printf("%-22s | %10.3f | %10.3f | %7.2fx\n",
                    "explode displace", aosMs, soaMs, aosMs / soaMs);
        
        translate(aos, 4 * OFFSET, 0.0f, 0.0f);
        savePLY("out_4_exploded.ply", aos);
    }
    
    std::cout << "\nAll 4 transformations saved as out_*.ply\n";
    return 0;
}