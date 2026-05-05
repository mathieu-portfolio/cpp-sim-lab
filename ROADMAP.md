# cpp-sim-lab Roadmap

Goal: explore systems programming through diverse simulations.
Focus: correctness first, measure before optimizing, minimal abstractions.

---

## Completed

### particles
- Particle system (position, velocity, radius)
- Gravity + integration
- Boundary collisions
- Particle collisions
- Spatial grid broad phase
- Benchmarks:
  - naive vs grid
  - grid cell size sweep
  - unordered_map vs flat grid

### boids
- Alignment, cohesion, separation
- Naive O(n²) neighbor search
- Spatial grid optimization
- Benchmarks:
  - naive vs grid
  - backend execution matrix

### agents_cpu
- Goal-driven agent intents
- Steering behavior composition
- Obstacle-aware movement
- Benchmarks:
  - spatial grid variants
  - obstacle scaling
  - parallel compare

### crowd
- Crowd movement and obstacle interactions
- Flow-field style local behavior

### sand_cpu
- Cellular-style granular updates
- Double-buffer style stepping and UI controls

### bubbles_cpu
- Bubble interactions and boundary behavior

### heat_grid_cpu
- 2D diffusion / heat propagation over dense grids
- boundary-condition handling and temperature-source injection

### predactor_prey_cpu
- predator-prey population dynamics over spatial agents
- local interaction rules (hunt, flee, reproduce, decay)

---

## Planned Simulations

### 1. cloth_cpu
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

### 2. fluids_grid_cpu
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

### 3. rigid_bodies_2d
Focus: advanced collision and physics

- Rotation and angular velocity
- Impulses with mass and inertia
- Contact resolution

Key learnings:
- Physics modeling
- Collision manifolds
- More complex math

---

### 4. cellular_automata_cpu
Focus: discrete systems

- Game of Life / falling sand / fire
- Double buffering
- Grid updates

Key learnings:
- Deterministic updates
- Memory traversal patterns

---

### 5. nbody_cpu
Focus: algorithmic complexity

- O(n²) baseline
- Barnes-Hut (quadtree)
- Accuracy vs performance

Key learnings:
- Tree structures
- Approximation techniques

---

### GPU readiness checkpoint

We should only start a GPU port when all of the following are true for a simulation:
- stable CPU feature set (no frequent behavior changes)
- deterministic enough tests to catch regressions
- benchmark coverage across entity-count scaling and backends
- dominant time spent in data-parallel hot loops (not UI or branching-heavy control)
- clear state layout suitable for SoA-style buffers

Current best candidates (ordered):
1. `particles`
   - strongest data-parallel structure (integration + collision passes)
   - existing execution matrix benchmark
2. `boids`
   - per-entity steering accumulation is GPU-friendly
   - existing execution matrix benchmark
3. `crowd`
   - many independent agent updates with shared spatial queries
   - existing execution matrix benchmark

Secondary candidates:
- `bubbles_cpu` (similar interaction profile to particles, but less benchmark depth)
- `traffic_flow_cpu` (has benchmark matrix, but lane-change logic is branchier)
- `heat_grid_cpu` (dense grid stencils are very GPU-friendly, pending backend matrix benchmarks)

Not yet ideal first GPU ports:
- `agents_cpu` (behavior + intent transitions add control divergence)
- `sand_cpu` (cell rule dependencies can require careful sync design)
- `epidemic_cpu` (currently has population scaling benchmark, but less backend-matrix coverage)
- `predactor_prey_cpu` (branch-heavy interaction rules and variable neighborhood work per agent)

### GPU project structure decision

Recommendation: **do not split into a separate GPU project**.
Keep one simulation per domain and expose `cpu`/`gpu` execution backends behind the same simulation entry point.

Why:
- preserves feature parity pressure (same UI, same scenario, same metrics)
- avoids duplicated product surface and drift between cpu/gpu variants
- keeps benchmarking apples-to-apples inside one binary/config space
- aligns with existing backend toggles (naive/grid, single-thread/parallel)

Naming convention:
- keep current simulation names (`particles`, `boids`, `crowd`, ...)
- use runtime/backend labels for execution mode (`cpu_scalar`, `cpu_parallel`, `gpu_compute`)
- reserve `_cpu` suffix in target names only as temporary migration detail

When to split anyway (exception):
- different numerical model is required on GPU
- platform/toolchain constraints make unified build too costly
- shared code drops below ~60% and maintenance overhead clearly rises

Incremental rollout plan:
1. Port `particles` first as `particles` with selectable backend.
2. Keep authoritative correctness checks on CPU; compare GPU statistically/tolerance-based.
3. Reuse same scene generators and benchmark harness for cpu/gpu runs.
4. After two successful ports, reevaluate whether unified structure still pays off.

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
