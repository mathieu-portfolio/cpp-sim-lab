#pragma once

#include "Cell.hpp"

#include <simulation/SimulationBase.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sand_cpu {

struct SimulationConfig {
    float width = 800.0f;
    float height = 800.0f;

    std::size_t gridWidth = 256;
    std::size_t gridHeight = 256;
    std::size_t chunkSize = 16;

    int brushRadius = 4;
    int spawnAmount = 20;
    std::size_t maxParticleCount = 65536;

    float cellSize = 3.0f;
    simfw::simulation::SimulationExecutionConfig execution{};
    std::size_t entityCount = 0;
};

struct SimulationStats {
    std::size_t particleCount = 0;
    std::size_t maxParticleCount = 0;
    std::size_t entityCount = 0;
    std::size_t activeChunks = 0;
    std::size_t movedCells = 0;
};

class Simulation : public simfw::SimulationBase<SimulationConfig, SimulationStats, Cell> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, Cell>;

    explicit Simulation(SimulationConfig config = {});

    void update(float dt);
    void reset();
    void clear();
    void spawnDisc(int centerX, int centerY, Material material);

    const std::vector<Cell>& getCells() const { return m_cells; }

private:
    std::vector<Cell> m_cells;
    std::vector<std::uint8_t> m_activeChunks;
    std::vector<std::uint8_t> m_nextActiveChunks;
    std::size_t m_chunkCols = 0;
    std::size_t m_chunkRows = 0;

    [[nodiscard]] std::size_t idx(int x, int y) const;
    [[nodiscard]] bool inBounds(int x, int y) const;
    [[nodiscard]] bool tryMove(int x, int y, int nx, int ny, bool markActivity = true);
    void markChunkDirtyByCell(int x, int y);
    void markNeighborsDirty(int x, int y);
    void rebuildStats();
};

} // namespace sand_cpu
