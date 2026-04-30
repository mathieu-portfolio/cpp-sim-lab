# Simulations

Each simulation is independent and demonstrates a different system.

## particles_cpu
- particle collisions
- spatial partitioning
- pipeline-based update (passes, not behaviors)

## boids_cpu
- flocking simulation
- composable steering behaviors:
  - alignment
  - cohesion
  - separation
  - wander

## agents_cpu
- goal-oriented agents
- behavior + intent system
- obstacle avoidance
- target seeking

## Common Features
- grid vs naive backend
- parallel vs single-thread
- camera navigation
- entity selection & debug UI

## traffic_flow_cpu
- lane-based traffic flow
- IDM longitudinal control
- simple MOBIL-style lane changes
- throughput / speed / queue metrics
