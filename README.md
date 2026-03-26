# Particle Filter Localization

A **Monte Carlo Localization** (MCL) simulation implemented in C++ that uses a **particle filter** to estimate a robot's position within a 2D environment. The robot is equipped with sonar-like ray sensors and navigates a scene composed of geometric primitives (rectangles and triangles). The particle filter iteratively converges a cloud of random position hypotheses toward the robot's true location by comparing simulated sensor readings against actual ones.

---

## Table of Contents

- [Overview](#overview)
- [How It Works](#how-it-works)
- [Architecture & File Descriptions](#architecture--file-descriptions)
- [Class Hierarchy](#class-hierarchy)
- [Algorithm Details](#algorithm-details)
- [Dependencies](#dependencies)
- [Build & Run](#build--run)
- [Configuration & Customization](#configuration--customization)
- [Example Output](#example-output)

---

## Overview

Localization is a fundamental problem in robotics: *"Where am I?"* This project answers that question using a **particle filter** — a probabilistic algorithm that maintains a set of hypothesized positions (particles) and refines them over time using sensor observations.

The simulation creates:
1. A **2D scene** (map) bounded by a rectangle.
2. A **robot** placed at the origin with multiple directional **ray sensors** (simulating sonar/LIDAR beams).
3. A **swarm of particles** (100 by default), each representing a candidate position for the robot.

At each time step the robot moves, its sensors fire, and the particle filter updates, reweights, and resamples the particles until the best particle converges to the robot's true position.

---

## How It Works

1. **Initialization** — 100 particles are uniformly scattered across the map.
2. **Prediction (Motion Update)** — All particles are shifted by the same displacement the robot moved (`dX`, `dY`), with toroidal wrapping at map boundaries.
3. **Sensor Measurement** — The robot's rays are cast into the scene; intersection distances with obstacles are recorded as the *real* sonar readings.
4. **Weight Update** — Each particle temporarily assumes its candidate position, casts the same rays, and compares its simulated readings against the real ones using a **Gaussian likelihood** function. Particles whose simulated readings closely match the real readings receive higher weights.
5. **Normalization** — All particle weights are normalized so they sum to 1, forming a probability distribution.
6. **Resampling** — Particles are redrawn from this distribution (with replacement). High-weight particles are duplicated; low-weight ones are discarded. A small Gaussian noise is added to maintain diversity and avoid particle collapse.
7. **Output** — The particle with the highest weight is printed as the best position estimate.

Steps 2–7 repeat for a configurable number of iterations (50 by default).

---

## Architecture & File Descriptions

| File | Description |
|---|---|
| **main1.cpp** | Entry point. Creates the scene, robot, and rays. Adds a bounding rectangle as the only obstacle. Runs the localization loop (`RobotTest`), which moves the robot, updates particles, and prints the estimated position at each step. |
| **Particles.h** | Core particle filter engine. Manages the particle population as a 2D array (`7 × N`): rows 0–1 are X/Y position, rows 2–5 are Z/roll/pitch/yaw (reserved for 3D extension), and row 6 stores the weight. Implements `initialize()`, `updateState()`, `updateWeights()`, `normalize()`, `resample()`, and `printExpectedPosition()`. |
| **Robot.h** | Represents the robot. Holds a 2D position (`pos`), a direction vector (`dir`), and a collection of `Ray` sensor pointers. Provides `AddRay()` to attach sensors. |
| **Ray.h** | Models a single ray/sonar beam. Stores an origin, a direction, an angular offset (converted to radians), and the hit distance `t` (initialized to `INFINITY` meaning "no hit"). |
| **Scene.h** | The environment/map. Maintains a list of geometric primitives (`Prim` objects). The `Trace()` method fires all of a robot's rays into the scene: for each ray it applies a 2D rotation matrix based on the ray's angle offset and the robot's facing direction, then tests intersection against every object, recording the nearest hit distance. Returns an array of distances. |
| **Prim.h** | Abstract base class for scene primitives. Declares a pure virtual `Intersect(Ray&)` method that concrete shapes must implement. Uses the non-copyable idiom. |
| **Rectangle.h** | Concrete primitive — an axis-aligned or arbitrary quadrilateral defined by four vertices. Intersection is computed by testing the ray against each of the four edges using a 2D cross-product parametric method. The closest valid intersection (`t > 0`, parameter in `[0,1]`) is kept. |
| **Triangle.h** | Concrete primitive — a triangle defined by three vertices. Intersection is computed by solving 2×2 linear systems (matrix inversion) for each pair of edges, checking parametric bounds, and keeping the nearest valid hit. |

---

## Class Hierarchy

```
Prim (abstract)
├── Rectangle
└── Triangle

Robot
├── has many → Ray (shared_ptr)
└── has → pos, dir

Scene
├── has many → Prim (shared_ptr)
└── Trace(Robot&) → fires rays, returns distances

Particles
├── has → Robot (internal copy for simulation)
├── has → Scene* (pointer to shared scene)
└── manages N particles (7-row × N-column array)
```

---

## Algorithm Details

### Weight Calculation (Gaussian Likelihood)

For each particle, the weight is computed as:

$$w_i = \frac{1}{K} \sum_{k=1}^{K} \frac{1}{\sigma \sqrt{2\pi}} \exp\left(-\frac{1}{2}\left(\frac{s_k^{\text{sim}} - s_k^{\text{real}}}{\sigma}\right)^2\right)$$

Where:
- $K$ = number of ray sensors
- $s_k^{\text{sim}}$ = simulated sonar reading for ray $k$ at the particle's position
- $s_k^{\text{real}}$ = actual sonar reading from the robot
- $\sigma = 0.5$ (standard deviation, controls sensitivity)

### Resampling

Uses **stochastic universal sampling** via cumulative probability. Selected particles are perturbed with Gaussian noise ($\mu = 0$, $\sigma = 0.5$) to maintain diversity and prevent degeneracy.

### Ray–Edge Intersection

For a ray with origin $\mathbf{o}$ and direction $\mathbf{d}$, intersection with an edge from $\mathbf{v_1}$ to $\mathbf{v_2}$ is found by solving:

$$\mathbf{o} + t\,\mathbf{d} = \mathbf{v_1} + u\,(\mathbf{v_2} - \mathbf{v_1})$$

Using the 2D cross product: $\mathbf{a} \times \mathbf{b} = a_x b_y - a_y b_x$

A valid intersection requires $t > 0$ and $0 < u < 1$.

---

## Dependencies

- **OpenCV** (core modules only) — used for `cv::Vec2f`, `cv::Mat`, and matrix operations (`inv()`, rotation matrices, normalization).
  - Tested with OpenCV 3.x / 4.x.
- **C++11** or later — uses `<random>`, `<chrono>`, `<memory>` (smart pointers), range-based for loops.
- A C++ compiler supporting C++11 (e.g., `g++`, `clang++`).

---

## Build & Run

### Install OpenCV (macOS)

```bash
brew install opencv
```

### Compile

```bash
g++ -std=c++11 -o main main1.cpp `pkg-config opencv4 --cflags --libs`
```

> If `opencv4` is not found, try `pkg-config opencv --cflags --libs` instead.

### Run

```bash
./main
```

---

## Configuration & Customization

Key parameters can be adjusted in the source code:

| Parameter | Location | Default | Description |
|---|---|---|---|
| Number of particles | `main1.cpp` → `Particles(...)` constructor | `100` | More particles = better accuracy but slower. |
| Iteration count | `main1.cpp` → `for` loop in `RobotTest` | `50` | Number of motion/sense/resample cycles. |
| Scene dimensions | `main1.cpp` → `Scene(100, 100)` | `100 × 100` | Width and height of the 2D map. |
| Robot displacement | `main1.cpp` → `r.pos[0]+=1` | `(1, 1)` per step | How far the robot moves each iteration. |
| Ray angles | `main1.cpp` → `Ray(45)`, `Ray(10)`, `Ray(-45)` | `45°, 10°, -45°` | Angular offsets of sonar beams relative to robot facing direction. |
| Gaussian σ (weights) | `Particles.h` → `updateWeights()` | `0.5` | Controls how sharply the weight peaks around matching readings. |
| Gaussian σ (resample noise) | `Particles.h` → `resample()` | `0.5` | Controls diversity of resampled particles. |
| Obstacles | `main1.cpp` → `scene.Add(...)` | Bounding rectangle only | Uncomment existing lines or add new `Triangle` / `Rectangle` objects. |

---

## Example Output

Each iteration prints the best particle's estimated position:

```
update weight finished
Best Particle (weight: 0.0523):
X: 3.12
Y: 2.87
Index: 42
...
50 50      ← robot's actual final position
```

As the filter converges, the best particle's `(X, Y)` should approach the robot's true position.

---

## Potential Extensions

- **Visualization** — Render the scene, robot, rays, and particle cloud using OpenCV's drawing functions or a library like SFML.
- **3D support** — The particle state already reserves dimensions for Z, roll, pitch, and yaw.
- **Dynamic obstacles** — Add moving primitives and update the scene between iterations.
- **Real sensor data** — Replace synthetic `Trace()` calls with actual LIDAR/sonar input.
- **Adaptive particle count** — KLD-sampling to dynamically adjust the number of particles based on approximation quality.
