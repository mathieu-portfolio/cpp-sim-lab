# cpp-sim-lab

A collection of CPU-based simulations used to explore:
- spatial partitioning (grid vs naive)
- parallel update patterns
- steering behaviors and agent systems

## Structure

- framework/
  Shared utilities (parallel update, spatial queries, UI, stats)

- simulations/
  Independent simulations built on the framework:
  - particles_cpu
  - boids_cpu
  - agents_cpu

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

- Start with a simulation in `simulations/`
- Look at its `Simulation.cpp`
- Then check corresponding behaviors (if any)
- Framework utilities are in `framework/`

## For AI / New Readers

If you're exploring:
- Entry point: `simulations/*/main.cpp`
- Simulation loop: `Simulation::update`
- Behavior logic: `*Behavior*.cpp`
- Shared systems: `framework/simulation/`
