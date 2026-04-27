#pragma once

#include "Particle.hpp"

#include <spatial/SpatialHashGrid.hpp>

#include <vector>

class SpatialGrid {
public:
    explicit SpatialGrid(float cellSize);

    void clear();
    void insert(int particleIndex, const Vec2& position);
    void build(const std::vector<Particle>& particles);

    void queryNeighbors(const Vec2& position, std::vector<int>& outIndices) const;

    const auto& getCells() const {
        return m_grid.getCells();
    }

private:
    simfw::SpatialHashGrid<int> m_grid;
};
