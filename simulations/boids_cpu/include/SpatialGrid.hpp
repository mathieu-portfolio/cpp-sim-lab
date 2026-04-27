#pragma once

#include "Boid.hpp"

#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <vector>

class SpatialGrid {
public:
    explicit SpatialGrid(float cellSize = 50.0f);

    void setCellSize(float cellSize);
    float getCellSize() const;

    void clear();
    void build(const std::vector<Boid>& boids);

    void queryNeighbors(
        Vec2 position,
        float radius,
        std::vector<std::size_t>& outIndices
    ) const;

    const auto& getCells() const {
        return m_grid.getCells();
    }

private:
    simfw::SpatialHashGrid<std::size_t> m_grid;
};
