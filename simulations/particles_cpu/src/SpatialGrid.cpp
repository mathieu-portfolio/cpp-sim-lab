#include "SpatialGrid.hpp"

SpatialGrid::SpatialGrid(float cellSize)
    : m_cellSize(cellSize) {}

void SpatialGrid::clear() {
    m_cells.clear();
}

CellCoord SpatialGrid::toCell(const Vec2& position) const {
    return CellCoord{
        static_cast<int>(position.x / m_cellSize),
        static_cast<int>(position.y / m_cellSize)
    };
}

void SpatialGrid::insert(int particleIndex, const Vec2& position) {
    CellCoord cell = toCell(position);
    m_cells[cell].push_back(particleIndex);
}

void SpatialGrid::build(const std::vector<Particle>& particles) {
    clear();

    for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
        insert(i, particles[i].position);
    }
}

void SpatialGrid::queryNeighbors(const Vec2& position, std::vector<int>& outIndices) const {
    CellCoord base = toCell(position);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            CellCoord neighbor{base.x + dx, base.y + dy};

            auto it = m_cells.find(neighbor);
            if (it != m_cells.end()) {
                outIndices.insert(outIndices.end(), it->second.begin(), it->second.end());
            }
        }
    }
}
