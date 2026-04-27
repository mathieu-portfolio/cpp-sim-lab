# cpp-sim-lab Roadmap

Goal: explore systems programming through diverse simulations.
Focus: correctness first, measure before optimizing, minimal abstractions.

---

## Completed

### particles_cpu
- Particle system (position, velocity, radius)
- Gravity + integration
- Boundary collisions
- Particle collisions
- Spatial grid broad phase
- Benchmarks:
  - naive vs grid
  - grid cell size sweep
  - unordered_map vs flat grid

---

## Planned Simulations

### 1. boids_cpu
Focus: local interactions and emergent behavior

- Alignment, cohesion, separation
- Naive O(n²) neighbor search
- Spatial grid optimization (reuse idea, not code yet)
- Benchmark: naive vs grid

Key learnings:
- Force accumulation
- Neighborhood queries
- Stability and tuning

---

### 2. cloth_cpu
Focus: constraints and iterative solvers

- Verlet integration
- Distance constraints
- Pinned particles
- Iterative constraint solving

Key learnings:
- Numerical stability
- Solver iteration trade-offs
- Graph-like data structures

---

### 3. fluids_grid_cpu
Focus: grid-based simulation

- Velocity field (2D grid)
- Advection, diffusion
- Pressure projection
- Boundary conditions

Key learnings:
- Dense memory layouts
- Cache behavior
- Numerical methods

---

### 4. rigid_bodies_2d
Focus: advanced collision and physics

- Rotation and angular velocity
- Impulses with mass and inertia
- Contact resolution

Key learnings:
- Physics modeling
- Collision manifolds
- More complex math

---

### 5. cellular_automata_cpu
Focus: discrete systems

- Game of Life / falling sand / fire
- Double buffering
- Grid updates

Key learnings:
- Deterministic updates
- Memory traversal patterns

---

### 6. nbody_cpu
Focus: algorithmic complexity

- O(n²) baseline
- Barnes-Hut (quadtree)
- Accuracy vs performance

Key learnings:
- Tree structures
- Approximation techniques

---

## Future (after multiple sims)

### Parallelization
- Multithreading (task-based, data parallel)
- SIMD
- GPU (optional later)

Rule:
Only parallelize after identifying real bottlenecks.

---

## Extraction Rule

Code moves to `framework/` only if:
- Used in at least 2 simulations
- Abstraction is obvious and stable

Avoid premature generalization.

---

## General Principles

- Start simple (naive implementation first)
- Add instrumentation before optimizing
- Benchmark competing approaches
- Prefer clarity over cleverness

