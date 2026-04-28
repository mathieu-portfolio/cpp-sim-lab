#pragma once

#include <math/Vec2.hpp>

#include <cmath>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

namespace simfw {

struct CellCoord {
    int x = 0;
    int y = 0;

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

template <typename Index = std::size_t>
class SpatialHashGrid {
public:
    explicit SpatialHashGrid(float cellSize = 50.0f)
        : m_cellSize(cellSize) {}

    void setCellSize(float cellSize) {
        m_cellSize = cellSize;
    }

    float getCellSize() const {
        return m_cellSize;
    }

    void clear() {
        m_cells.clear();
    }

    void insert(Index objectIndex, const Vec2& position) {
        m_cells[toCell(position)].push_back(objectIndex);
    }

    template <typename Objects, typename PositionFn>
    void build(const Objects& objects, PositionFn getPosition) {
        clear();

        for (std::size_t i = 0; i < objects.size(); ++i) {
            insert(static_cast<Index>(i), getPosition(objects[i]));
        }
    }

    void queryCellsAround(
        const Vec2& position,
        int cellRadius,
        std::vector<Index>& outIndices
    ) const {
        const CellCoord base = toCell(position);

        for (int dy = -cellRadius; dy <= cellRadius; ++dy) {
            for (int dx = -cellRadius; dx <= cellRadius; ++dx) {
                const CellCoord coord{base.x + dx, base.y + dy};

                auto it = m_cells.find(coord);
                if (it == m_cells.end()) {
                    continue;
                }

                outIndices.insert(
                    outIndices.end(),
                    it->second.begin(),
                    it->second.end()
                );
            }
        }
    }

    void queryRadius(
        const Vec2& position,
        float radius,
        std::vector<Index>& outIndices
    ) const {
        const int cellRadius =
            static_cast<int>(std::ceil(radius / m_cellSize));

        queryCellsAround(position, cellRadius, outIndices);
    }

    const std::unordered_map<CellCoord, std::vector<Index>, CellCoordHash>& getCells() const {
        return m_cells;
    }

private:
    float m_cellSize;
    std::unordered_map<CellCoord, std::vector<Index>, CellCoordHash> m_cells;

    CellCoord toCell(const Vec2& position) const {
        return CellCoord{
            static_cast<int>(std::floor(position.x / m_cellSize)),
            static_cast<int>(std::floor(position.y / m_cellSize))
        };
    }
};

} // namespace simfw
