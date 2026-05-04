#pragma once

namespace simfw::simulation {

enum class ComputeBackend {
    CpuScalar,
    CpuParallel,
    GpuCompute
};

struct SimulationExecutionConfig {
    bool useSpatialGrid = true;
    bool useParallelUpdate = true;
    ComputeBackend computeBackend = ComputeBackend::CpuParallel;
};

inline const char* computeBackendName(ComputeBackend backend) {
    switch (backend) {
    case ComputeBackend::CpuScalar:
        return "cpu_scalar";
    case ComputeBackend::CpuParallel:
        return "cpu_parallel";
    case ComputeBackend::GpuCompute:
        return "gpu_compute";
    }

    return "unknown";
}

} // namespace simfw::simulation
