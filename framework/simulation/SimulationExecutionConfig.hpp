#pragma once

namespace simfw::simulation {

struct SimulationExecutionConfig {
    bool useSpatialGrid = true;
    bool useParallelUpdate = true;
};

} // namespace simfw::simulation
