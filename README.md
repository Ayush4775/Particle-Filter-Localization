# Particle Filter Localization

A C++ Monte Carlo Localization (MCL) simulation using a particle filter to estimate a robot's 2D position. The robot uses ray sensors to sense obstacles (rectangles/triangles), and particles converge toward the true position by comparing simulated vs. actual sensor readings.

## How It Works

1. Scatter particles randomly across the map
2. Move all particles by the robot's displacement
3. Cast rays from the robot and each particle
4. Weight particles by how closely their readings match the robot's (Gaussian likelihood)
5. Resample — duplicate high-weight particles, discard low-weight ones, add noise for diversity
6. Repeat

## Files

| File | Role |
|---|---|
| `main1.cpp` | Entry point — sets up scene, robot, rays, and runs the localization loop |
| `Particles.h` | Particle filter: initialize, predict, weight, normalize, resample |
| `Robot.h` | Robot with position, direction, and ray sensors |
| `Ray.h` | Single ray/sonar beam |
| `Scene.h` | Map of primitives; traces rays and returns hit distances |
| `Prim.h` | Abstract base for scene geometry |
| `Rectangle.h` | 4-vertex shape with ray–edge intersection |
| `Triangle.h` | 3-vertex shape with ray–edge intersection |

## Build & Run

Requires **OpenCV** and **C++11**.

```bash
brew install opencv
g++ -std=c++11 -o main main1.cpp `pkg-config opencv4 --cflags --libs`
./main
```

## Key Parameters (in source)

- **Particle count** — `Particles(...)` constructor in `main1.cpp` (default: 100)
- **Iterations** — loop count in `main1.cpp` (default: 50)
- **Scene size** — `Scene(100, 100)`
- **Ray angles** — e.g. `Ray(45)`, `Ray(10)`, `Ray(-45)`
- **Gaussian σ** — in `Particles.h` (default: 0.5 for both weights and resample noise)
