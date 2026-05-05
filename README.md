# cpp-sim-lab

A collection of CPU-based simulations used to explore:
- spatial partitioning (grid vs naive)
- parallel update patterns
- steering behaviors and agent systems

## Setup

### Requirements

- CMake 3.16+
- C++20 compatible compiler
- vcpkg
- Ninja (recommended on Windows / Git Bash)

### Dependencies

Dependencies are managed through vcpkg.json.

Current dependencies:
- raylib

Set VCPKG_ROOT before configuring:

```bash
export VCPKG_ROOT=/c/Users/Mathieu/vcpkg
```

On Windows (Git Bash), prefer Ninja:

```bash
cmake --preset debug-ninja
cmake --build --preset debug-ninja
```

### Build & Run

Build and run a simulation:

```bash
scripts/run_sim.sh crowd_cpu debug-ninja --build
```

Or using default preset:

```bash
scripts/run_sim.sh crowd_cpu --build
```

If raylib is not found:

```bash
"$VCPKG_ROOT/vcpkg.exe" install raylib:x64-windows
```

Then clean and reconfigure:

```bash
rm -rf build/debug-ninja
cmake --preset debug-ninja
```

## Structure

- framework/
  Shared utilities (parallel update, spatial queries, UI, stats)

- simulations/
  Independent simulations built on the framework:
  - particles
  - boids
  - agents_cpu
  - crowd_cpu
  - sand_cpu
  - bubbles_cpu

- benchmarks/
  Performance comparisons (grid vs naive, parallel vs single-thread)

- tests/
  Unit and simulation tests

## Simulations Overview

| Simulation   | Focus                          | Key Concepts                     |
|--------------|-------------------------------|----------------------------------|
| particles    | physics + collisions          | spatial grid, collision passes   |
| boids        | flocking                      | composable behaviors             |
| agents       | goal-driven agents            | behaviors + intent system        |
| crowd        | crowd navigation              | flow fields, obstacle handling   |
| sand         | cellular material updates     | grid stepping, local rules       |
| bubbles      | soft-body style interactions  | collisions, simple fluid feel    |

## Key Concepts

### Execution Backends
All simulations support:
- naive vs spatial grid
- single-thread vs parallel

### Behavior System (boids, agents)
- behaviors are composable
- shared weighted pipeline
- pure functions + runtime wrappers

### Camera & Interaction
- zoom / pan
- entity selection
- debug overlays

## How to Navigate

- Start with a simulation in simulations/
- Look at its Simulation.cpp
- Then check corresponding behaviors (if any)
- Framework utilities are in framework/

## For AI / New Readers

If you're exploring:
- Entry point: simulations/*/main.cpp
- Simulation loop: Simulation::update
- Behavior logic: *Behavior*.cpp
- Shared systems: framework/simulation/
