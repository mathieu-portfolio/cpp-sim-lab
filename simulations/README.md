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


## predator_prey_cpu
- two-population boids (prey + predators)
- prey flocking plus predator avoidance
- predator nearest-target chase and catch
- simple predator energy drain/respawn loop

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

## epidemic_cpu
- SIR-style epidemic spread over moving agents
- proximity/contact-time infection probability
- recovery timer and epidemic metrics (R_t proxy, peak infection, extinction time)


## heat_grid_cpu
- double-buffered 2D heat diffusion grid
- 5-point stencil update
- fixed heat sources/sinks
- boundary mode toggle (clamp/wrap/insulated)
