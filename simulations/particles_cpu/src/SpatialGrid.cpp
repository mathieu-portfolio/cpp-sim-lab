#include "SpatialGrid.hpp"

SpatialGrid::SpatialGrid(float cellSize)
    : m_grid(cellSize) {}

void SpatialGrid::clear() {
    m_grid.clear();
}

void SpatialGrid::insert(int particleIndex, const Vec2& position) {
    m_grid.insert(particleIndex, position);
}

void SpatialGrid::build(const std::vector<Particle>& particles) {
    m_grid.build(
        particles,
        [](const Particle& particle) {
            return particle.position;
        }
    );
}

void SpatialGrid::queryNeighbors(const Vec2& position, std::vector<int>& outIndices) const {
    m_grid.queryCellsAround(position, 1, outIndices);
}
