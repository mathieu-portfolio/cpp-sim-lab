#pragma once

#include <math/Vec2.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace simfw::simulation {

enum class ObstaclePaintMode { Block, Erase };

class ObstacleMask {
public:
    ObstacleMask() = default;
    ObstacleMask(int width, int height) { resize(width, height); }

    void resize(int width, int height) {
        m_width = std::max(0, width);
        m_height = std::max(0, height);
        m_cells.assign(static_cast<std::size_t>(m_width * m_height), 0u);
    }

    void clear() { std::fill(m_cells.begin(), m_cells.end(), 0u); }

    [[nodiscard]] int width() const { return m_width; }
    [[nodiscard]] int height() const { return m_height; }
    [[nodiscard]] bool empty() const { return m_width == 0 || m_height == 0 || m_cells.empty(); }

    [[nodiscard]] bool isBlocked(int x, int y) const {
        if (!inBounds(x, y)) {
            return false;
        }
        return m_cells[indexOf(x, y)] != 0u;
    }

    void setBlocked(int x, int y, bool blocked) {
        if (!inBounds(x, y)) {
            return;
        }
        m_cells[indexOf(x, y)] = blocked ? 1u : 0u;
    }

    bool paintCircle(Vec2 center, float radius, ObstaclePaintMode mode) {
        if (empty()) {
            return false;
        }

        const int minX = std::max(0, static_cast<int>(std::floor(center.x - radius)));
        const int maxX = std::min(m_width - 1, static_cast<int>(std::ceil(center.x + radius)));
        const int minY = std::max(0, static_cast<int>(std::floor(center.y - radius)));
        const int maxY = std::min(m_height - 1, static_cast<int>(std::ceil(center.y + radius)));
        const float radiusSq = radius * radius;

        bool changed = false;
        const uint8_t value = mode == ObstaclePaintMode::Block ? 1u : 0u;
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const float dx = static_cast<float>(x) - center.x;
                const float dy = static_cast<float>(y) - center.y;
                if (dx * dx + dy * dy > radiusSq) {
                    continue;
                }
                auto& cell = m_cells[indexOf(x, y)];
                if (cell != value) {
                    cell = value;
                    changed = true;
                }
            }
        }
        return changed;
    }

    [[nodiscard]] const std::vector<uint8_t>& cells() const { return m_cells; }
    [[nodiscard]] std::vector<uint8_t>& cells() { return m_cells; }

private:
    [[nodiscard]] bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < m_width && y < m_height; }
    [[nodiscard]] std::size_t indexOf(int x, int y) const { return static_cast<std::size_t>(y * m_width + x); }

    int m_width = 0;
    int m_height = 0;
    std::vector<uint8_t> m_cells;
};

} // namespace simfw::simulation
