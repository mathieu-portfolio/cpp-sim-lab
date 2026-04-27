#include "SpatialGrid.hpp"

SpatialGrid::SpatialGrid(float cellSize)
    : m_grid(cellSize) {}

void SpatialGrid::setCellSize(float cellSize) {
    m_grid.setCellSize(cellSize);
}

float SpatialGrid::getCellSize() const {
    return m_grid.getCellSize();
}

void SpatialGrid::clear() {
    m_grid.clear();
}

void SpatialGrid::build(const std::vector<Boid>& boids) {
    m_grid.build(
        boids,
        [](const Boid& boid) {
            return boid.position;
        }
    );
}

void SpatialGrid::queryNeighbors(
    Vec2 position,
    float radius,
    std::vector<std::size_t>& outIndices
) const {
    outIndices.clear();
    m_grid.queryRadius(position, radius, outIndices);
}
