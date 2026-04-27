#include "SpatialGrid.hpp"

#include <cmath>

SpatialGrid::SpatialGrid(float cellSize)
    : m_cellSize(cellSize) {}

void SpatialGrid::clear() {
    m_cells.clear();
}

CellCoord SpatialGrid::toCell(Vec2 position) const {
    return CellCoord{
        static_cast<int>(std::floor(position.x / m_cellSize)),
        static_cast<int>(std::floor(position.y / m_cellSize))
    };
}

void SpatialGrid::build(const std::vector<Boid>& boids) {
    clear();

    for (std::size_t i = 0; i < boids.size(); ++i) {
        m_cells[toCell(boids[i].position)].push_back(i);
    }
}

void SpatialGrid::queryNeighbors(
    Vec2 position,
    float radius,
    std::vector<std::size_t>& outIndices
) const {
    outIndices.clear();

    const CellCoord base = toCell(position);
    const int cellRadius = static_cast<int>(std::ceil(radius / m_cellSize));

    for (int dy = -cellRadius; dy <= cellRadius; ++dy) {
        for (int dx = -cellRadius; dx <= cellRadius; ++dx) {
            CellCoord coord{base.x + dx, base.y + dy};

            auto it = m_cells.find(coord);
            if (it == m_cells.end()) {
                continue;
            }

            outIndices.insert(outIndices.end(), it->second.begin(), it->second.end());
        }
    }
}