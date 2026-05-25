\# Point Cloud Manipulation and NeRF Pipeline



A C++ and Python project exploring point cloud transformations,

particle-based spatial simulations, and Neural Radiance Field rendering

pipelines.


\---



This repository is organized into three independent parts:



\- \*\*Part 1\*\*: Point cloud transformations on a sample from 8i Voxelized

&#x20; Full Bodies dataset (Loot.ply, \~784k points), comparing

&#x20; Array-of-Structures and Structure-of-Arrays representations.



\- \*\*Part 2\*\*: 2D particle simulation with naive O(N²) and spatial-hash

&#x20; O(N) collision detection, including a benchmark sweep.



\- \*\*Part 3\*\*: Analysis of NVlabs/instant-ngp source code, plus a practical

&#x20; NeRF training and mesh export using the fox dataset bundled with

&#x20; instant-ngp.



All C++ code uses C++17 and was developed and tested with g++ (MinGW) on

Windows. Part 3's practical export uses Google Colab with a Tesla T4 GPU.



\## Repository structure

.

├── part1/

│   ├── PointCloud.h / .cpp     # AoS + SoA implementations

│   ├── main.cpp                # Driver: 4 transformations + benchmark

│   ├── Loot.ply                # Input dataset

│   └── out\_\*.ply               # Output point clouds (after running)

│

├── part2/

│   ├── ParticleSystem.h / .cpp # integrate + collisions (naive + hashed)

│   ├── main.cpp                # 800-particle simulation, writes frames.csv

│   ├── benchmark.cpp           # Naive vs hashed scaling sweep

│   ├── viewer.html             # Browser-based animation viewer

│   └── benchmark.html          # Plot of benchmark.csv

│

├── part3/

│   ├── part3_report.pdf             # Theoretical analysis (a, b, c)

│   ├── instant-ngp.ipynb       # Colab notebook for build + train + export

│   ├── fox\_mesh.ply            # Marching cubes mesh output (not in repo, see below)

│   └── screenshots/            # Visual results

│

└── README.md                   # This file



\## Part 1: Point cloud transformations



\### What's done



In this part, we load the provided point cloud file (.ply format), then

apply four transformations: translation, axis-aligned rotation around Y,

a tilted-axis rotation via Rodrigues' formula, and an explode-style

displacement that pushes each point outward from the centroid.



Two data layouts are implemented in parallel: Array-of-Structures (AoS,

a single `vector<Point>`) and Structure-of-Arrays (SoA, six parallel

arrays for x, y, z, r, g, b). The driver applies all transformations one

by one and measures their timings under both layouts. Output point

clouds are saved as `out\_\*.ply` and verified visually in CloudCompare.



\### Build \& run



From inside `part1/`, with `Loot.ply` placed in the same directory:



```bash

g++ -O2 -std=c++17 PointCloud.cpp main.cpp -o test.exe

./test.exe

```



The program will:

1\. Load `Loot.ply`

2\. Apply each transformation under both AoS and SoA layouts

3\. Print a side-by-side timing table

4\. Write `out\_1\_translated.ply` through `out\_4\_exploded.ply` for inspection



\### Design decisions



\*\*Storage type for RGB.\*\* Each color channel is stored as `unsigned char`

(1 byte), matching the PLY spec for `uchar red/green/blue`. This keeps

each point at 15 bytes (12 for xyz + 3 for RGB), four times smaller than

using `int`, which improves cache locality during transformation hot loops.



\*\*Reading RGB.\*\* A subtlety in C++: passing an `unsigned char` to

`operator>>` makes the stream read a single character, not a parsed

integer. For example, the token `133` in the PLY file would be read as

the character `'1'` (ASCII 49). To avoid this, RGB values are read into

an `int` first and then narrowed via `static\_cast<unsigned char>`.



\*\*AoS as default, SoA as comparison.\*\* Array-of-Structures

(`std::vector<Point>`) is the natural and most readable layout for point

cloud data, where each point is a logical unit. SoA (six parallel arrays)

typically gives better performance on hot loops that only touch xyz

because (a) RGB does not get pulled into cache and (b) contiguous float

arrays are easier for the compiler to vectorize. Both layouts are

implemented in this code and compared in the benchmark below.



\*\*Rodrigues vs quaternion.\*\* Rotations are computed via Rodrigues' formula

into a 3x3 matrix, then applied to each point. Quaternions would offer

advantages for chaining many rotations (numerical stability, smaller

state), but here each rotation is independent and applied in one pass,

so a direct matrix is simpler and equally efficient.



\*\*In-place transformations.\*\* All transformation functions operate on the

input vector in place rather than returning a new copy. For 784k points

(\~12 MB), this avoids a full allocation+copy per transformation. The

driver makes one explicit copy per transformation before timing it so the

input remains unmodified.



\*\*Aliasing in rotation.\*\* The `applyMatrix` loop computes new x, y, z into

local variables before writing them back to the point, since otherwise the

second component would be computed from the already-modified first.



\### Performance notes



All measurements on Loot.ply (784,142 points), MinGW g++ 15.2 on Windows.



\*\*Effect of compiler optimization (AoS layout):\*\*



| Operation        | -O0      | -O2      | Speedup |

|------------------|---------:|---------:|--------:|

| translate        |  5.99 ms |  1.63 ms |   3.7×  |

| rotate Y (90°)   | 20.55 ms |  3.35 ms |   6.1×  |

| rotate tilted    | 15.66 ms |  3.43 ms |   4.6×  |

| explode displace | 12.89 ms |  5.30 ms |   2.4×  |



A single `-O2` flag delivers 2-6× speedup with no code changes; primarily

through auto-vectorization (SIMD), function inlining, and loop unrolling.



**AoS vs SoA at -O2 (median of 10 runs):**

| Operation          | AoS (ms) | SoA (ms) | Speedup |
|--------------------|---------:|---------:|--------:|
| translate          |    1.79  |    1.47  |   1.22× |
| rotate Y (90°)     |    1.96  |    1.66  |   1.18× |
| rotate tilted (45°)|    2.03  |    1.85  |   1.10× |
| explode displace   |    4.08  |    3.86  |   1.06× |


AoS to SoA conversion overhead: \~4-9 ms one-time cost.



SoA consistently outperforms AoS by 1.06×–1.22× across all operations
(median of 10 runs). The gains are modest but consistent and the working
set (~12 MB) fits in L3 cache, so the bottleneck is compute rather than memory bandwidth, limiting SoA's main advantage. Larger datasets
(>L3 cache) would likely show more pronounced SoA gains.



\### Limitations and future work



\- \*\*ASCII PLY only.\*\* Loading Loot.ply takes \~6.5 seconds, dominated by

&#x20; per-token string-to-float parsing. A binary PLY path would likely

&#x20; bring this down by 50× or more.

\- \*\*Single-run timings.\*\* A more rigorous benchmark would run each

&#x20; transformation many times with explicit cache warmup and report median.

\- \*\*No GPU path.\*\* At 784k points the CPU is fast enough. For datasets

&#x20; in the tens of millions of points, a CUDA kernel would be the natural

&#x20; next step.

\- \*\*No SIMD intrinsics.\*\* The code relies entirely on compiler

&#x20; auto-vectorization. Explicit SSE/AVX intrinsics would give more

&#x20; predictable performance but at the cost of portability and readability.



\## Part 2: Particle simulation



\### What's done



A 2D box containing N circular particles, each with position (x, y) and

velocity (vx, vy). Each simulation step performs three operations:

explicit Euler integration, wall collision (clamp to box + reflect normal

velocity), and particle-particle collision.



Particle collision is implemented in two ways:



\- \*\*Naive O(N²)\*\*: every pair is checked.

\- \*\*Spatial-hash O(N)\*\*: particles are placed into a uniform grid with

&#x20; cell size equal to one particle diameter (2r), and only same-cell plus

&#x20; adjacent-cell pairs are tested.



The initial placement uses rejection sampling to guarantee no two

particles start in overlap. The main driver simulates 800 particles for

600 steps and writes each frame to `frames.csv`, which can be loaded by

`viewer.html` for animated playback. A separate `benchmark.cpp` sweeps

N from 200 to 12800 and writes timings to `benchmark.csv`, which

`benchmark.html` plots on a log-scale chart.



\### Build \& run



From inside `part2/`:



```bash

\# Main simulation (writes frames.csv)

g++ -O2 -std=c++17 ParticleSystem.cpp main.cpp -o sim.exe

./sim.exe



\# Benchmark sweep (writes benchmark.csv)

g++ -O2 -std=c++17 ParticleSystem.cpp benchmark.cpp -o bench.exe

./bench.exe

```



To view the results, open `viewer.html` (for the animation) or

`benchmark.html` (for the scaling plot) in any modern browser, then

load the corresponding CSV file.



\### Design decisions



\*\*Cell size = 2r (one particle diameter).\*\* This is the smallest grid

size for which checking same-cell plus 8 adjacent cells is sufficient.

With a smaller cell (say `r`), two colliding particles could be 2 cells

apart, requiring a 5×5 patch scan (24 neighbors). With a larger cell

(say `4r`), neighbor scan stays simple but the average number of

particles per cell grows quadratically, and same-cell pair count

approaches O(N²) again. `cell = 2r` is the optimal value.



\*\*Forward-only neighbor scan.\*\* When iterating cells, only 4 of the 8

neighbors are checked (right, top-right, top, top-left). The other 4 are

not needed since every cell is the back-neighbor of some other cell, so each

unordered pair is visited exactly once. This halves the work without

changing correctness.



\*\*Positional correction + velocity swap.\*\* When two particles overlap,

both their positions (push apart by half the overlap each) and velocities

(swap normal components) are updated. Position correction alone leaves

particles still overlapping; velocity swap alone causes vibrating clumps

because the resolver keeps swapping back. Both are needed.



\*\*Separation check (`vaN - vbN <= 0`).\*\* After computing relative

velocity in the collision normal direction, we skip the velocity swap

if the particles are already separating. Without this, particles in a

nearly-resolved overlap would re-collide on the next step, creating the

same vibrating artifact.



\*\*Rejection sampling at spawn time.\*\* The initial placement rejects any

candidate that overlaps with an already-placed particle. This is O(N²)

but only runs once at initialization, so it doesn't affect per-frame

cost. Without it, the first frame would resolve initial overlaps and

the simulation could start with visible artifacts.



### Performance notes

Benchmark on a single CPU thread, MinGW g++ 15.2, density kept constant
(~25%) by scaling box ∝ √N:

| N     | Naive O(N²) | Spatial Hash O(N) | Speedup |
|-------|-------------|-------------------|---------|
| 200   | 0.034 ms    | 0.039 ms          | 0.9×    |
| 400   | 0.074 ms    | 0.054 ms          | 1.4×    |
| 800   | 0.297 ms    | 0.161 ms          | 1.9×    |
| 1600  | 1.135 ms    | 0.220 ms          | 5.2×    |
| 3200  | 4.402 ms    | 0.419 ms          | 10.5×   |

The crossover is around N=400-500. Below N=400, the spatial hash's grid
build overhead exceeds the savings. Above N=800, the gap widens
quadratically as expected.

N=6400 and above were excluded from the benchmark because the fixed box
size reaches physical packing density limits, causing `spawnParticles` to
fail. The remaining range N=200-3200 clearly demonstrates the algorithmic
scaling.

At N=3200, the naive approach is 10.5× slower and would prevent real-time
simulation; the hashed version still fits comfortably in a single frame
budget.


\### Limitations and future work



\- \*\*2D only.\*\* Extending to 3D is straightforward; spatial hash

&#x20; generalizes to a 3D grid, neighbor scan becomes 27 cells (or 13 with

&#x20; forward-only), collision math stays the same.

\- \*\*No rendering loop.\*\* The driver writes a CSV and the viewer is a

&#x20; separate HTML page. A real-time renderer (SFML, raylib) would tighten

&#x20; the dev loop but the test brief explicitly de-emphasized rendering.

\- \*\*Single-threaded.\*\* The grid build and pair check could be

&#x20; parallelized and counting-sort style grid build with prefix sum + scatter,

&#x20; then thread-per-cell collision resolution. This is the natural GPU

&#x20; pattern.

\- \*\*No restitution coefficient.\*\* Wall and particle collisions are

&#x20; perfectly elastic; real materials have e < 1.



\## Part 3: NeRF analysis and practical export



\### Theoretical analysis



See `report.docx` for the written analysis of the three questions:



a) Where does the rendering step take place in NVlabs/instant-ngp

b) What algorithms are used to infer the missing surfaces

c) How is view synthesis computed



The report walks through `src/testbed\_nerf.cu`, specifically the

`Testbed::render\_nerf` function and the `NerfTracer::trace` loop, and

explains how the three main contributions of the instant-ngp paper

(multi-resolution hash encoding, occupancy grid acceleration, and

fully-fused MLP) combine to enable near-real-time NeRF rendering.



\### Practical export



The notebook `instant-ngp.ipynb` documents the full pipeline:



1\. Clone and build NVlabs/instant-ngp from source (Colab T4 GPU)

2\. Train a NeRF on the fox dataset (15000 steps, \~5 minutes)

3\. Extract a mesh via marching cubes (resolution 256³, density

&#x20;  threshold 2.5)

4\. Save as `fox\_mesh.ply`



The output mesh shows the characteristic fox figure clearly recognizable,

with some surrounding "puffy" artifacts typical of marching-cubes

extraction from a continuous density field (the threshold choice trades

off detail completeness against noise (see screenshots) ).



\*\*Note on file size\*\*: `fox\_mesh.ply` is \~300 MB and exceeds GitHub's

file size limit. It can be regenerated by re-running the notebook, or

is available on request.



\### Setup notes



The Colab notebook handles all setup (CUDA toolchain, CMake, build).

A snapshot of the build was backed up to Google Drive partway through to

survive runtime disconnections. This is a useful trick for long-running Colab

sessions.



\## Closing notes



The brief mentioned that performance is a constraint for real-time

applications. Throughout this submission I tried to keep that lens

explicit: timing every operation in Part 1, characterizing the algorithmic

scaling in Part 2, and discussing the architectural decisions in

instant-ngp that make near-real-time NeRF possible in Part 3.



Limitations are listed honestly per part. Improvements I considered but

did not implement are noted as future work. None of them are blockers

for the test brief, and time was the main constraint.

