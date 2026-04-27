#pragma once

#include "Boid.hpp"

#include <cstddef>
#include <unordered_map>
#include <vector>

struct CellCoord {
    int x;
    int y;

    bool operator==(const CellCoord& other) const {
        return x == other.x && y == other.y;
    }
};

struct CellCoordHash {
    std::size_t operator()(const CellCoord& c) const noexcept {
        return (std::hash<int>()(c.x) * 73856093) ^
               (std::hash<int>()(c.y) * 19349663);
    }
};

class SpatialGrid {
public:
    explicit SpatialGrid(float cellSize = 50.0f);

    void setCellSize(float cellSize);
    float getCellSize() const { return m_cellSize; }

    void clear();
    void build(const std::vector<Boid>& boids);

    void queryNeighbors(
        Vec2 position,
        float radius,
        std::vector<std::size_t>& outIndices
    ) const;

    const std::unordered_map<CellCoord, std::vector<std::size_t>, CellCoordHash>& getCells() const {
        return m_cells;
    }

private:
    float m_cellSize;
    std::unordered_map<CellCoord, std::vector<std::size_t>, CellCoordHash> m_cells;

    CellCoord toCell(Vec2 position) const;
};