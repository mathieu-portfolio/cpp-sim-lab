#pragma once

#include "Particle.hpp"
#include <unordered_map>
#include <vector>
#include <cstdint>

struct CellCoord {
    int x;
    int y;

    bool operator==(const CellCoord& other) const {
        return x == other.x && y == other.y;
    }
};

struct CellCoordHash {
    std::size_t operator()(const CellCoord& c) const noexcept {
        // simple 2D hash
        return (std::hash<int>()(c.x) * 73856093) ^ (std::hash<int>()(c.y) * 19349663);
    }
};

class SpatialGrid {
public:
    explicit SpatialGrid(float cellSize);

    void clear();
    void insert(int particleIndex, const Vec2& position);
    void build(const std::vector<Particle>& particles);

    void queryNeighbors(const Vec2& position, std::vector<int>& outIndices) const;

    const std::unordered_map<CellCoord, std::vector<int>, CellCoordHash>& getCells() const {
        return m_cells;
    }

private:
    float m_cellSize;
    std::unordered_map<CellCoord, std::vector<int>, CellCoordHash> m_cells;

    CellCoord toCell(const Vec2& position) const;
};
