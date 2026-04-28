# Framework

Shared utilities used by all simulations.

## Contains

### simulation/
- ParallelUpdate
- SpatialQuery
- StatsReduction
- WeightedBehaviorPipeline

### ui/
- camera utilities
- debug UI
- tunable controls

## Design Philosophy

The framework provides:
- generic execution systems
- minimal abstractions

It does NOT define:
- simulation logic
- behaviors
- entity types

Those stay in each simulation.
