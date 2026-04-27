# particles_cpu

CPU particle simulation used to explore collision systems and performance trade-offs.

## Controls

```text
Left mouse: spawn  
Right mouse: clear  
R: reset  
Space: pause  
N: step  
G: toggle grid  

Mouse wheel: zoom  
Middle mouse: pan  
Backspace: reset camera  
```

## Overview

- Particle integration (gravity, damping)
- Boundary collisions
- Particle collisions
- Spatial grid broad phase
- Debug grid + occupancy visualization

## Benchmarks

- particles_collision_compare: naive vs grid
- particles_grid_cell_size: cell size sweep (build vs query cost)
- particles_fixed_grid_compare: unordered_map vs flat grid

## Run benchmark

```bash
./build/benchmarks/particles_grid_cell_size/particles_grid_cell_size_bench > results.csv  
python benchmarks/particles_grid_cell_size/plot.py results.csv  
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```