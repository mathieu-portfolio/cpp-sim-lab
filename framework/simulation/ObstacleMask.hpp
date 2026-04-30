#pragma once

#include <math/Vec2.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

namespace simfw::simulation {

inline bool paintMaskCircle(
    std::vector<uint8_t>& mask,
    int width,
    int height,
    Vec2 center,
    float radius
) {
    if (width <= 0 || height <= 0 || static_cast<int>(mask.size()) != width * height) {
        return false;
    }

    const int minX = std::max(0, static_cast<int>(std::floor(center.x - radius)));
    const int maxX = std::min(width - 1, static_cast<int>(std::ceil(center.x + radius)));
    const int minY = std::max(0, static_cast<int>(std::floor(center.y - radius)));
    const int maxY = std::min(height - 1, static_cast<int>(std::ceil(center.y + radius)));
    const float radiusSq = radius * radius;

    bool changed = false;
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const float dx = static_cast<float>(x) - center.x;
            const float dy = static_cast<float>(y) - center.y;
            if (dx * dx + dy * dy > radiusSq) {
                continue;
            }
            auto& cell = mask[static_cast<std::size_t>(y * width + x)];
            if (cell == 0u) {
                cell = 1u;
                changed = true;
            }
        }
    }
    return changed;
}

inline std::vector<std::pair<Vec2, float>> buildRigidBodyCirclesFromMask(
    const std::vector<uint8_t>& mask,
    int width,
    int height
) {
    std::vector<std::pair<Vec2, float>> circles;
    if (width <= 0 || height <= 0 || static_cast<int>(mask.size()) != width * height) {
        return circles;
    }

    std::vector<uint8_t> visited(mask.size(), 0u);
    auto indexOf = [width](int x, int y) { return static_cast<std::size_t>(y * width + x); };

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t startIndex = indexOf(x, y);
            if (visited[startIndex] != 0u || mask[startIndex] == 0u) {
                continue;
            }
            std::queue<std::pair<int, int>> queue;
            std::vector<std::pair<int, int>> componentPixels;
            visited[startIndex] = 1u;
            queue.push({x, y});

            while (!queue.empty()) {
                const auto [cx, cy] = queue.front();
                queue.pop();
                componentPixels.push_back({cx, cy});

                for (int oy = -1; oy <= 1; ++oy) {
                    for (int ox = -1; ox <= 1; ++ox) {
                        if (ox == 0 && oy == 0) {
                            continue;
                        }
                        const int nx = cx + ox;
                        const int ny = cy + oy;
                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                            continue;
                        }
                        const std::size_t nIndex = indexOf(nx, ny);
                        if (visited[nIndex] != 0u || mask[nIndex] == 0u) {
                            continue;
                        }
                        visited[nIndex] = 1u;
                        queue.push({nx, ny});
                    }
                }
            }

            float cx = 0.0f;
            float cy = 0.0f;
            for (const auto& [px, py] : componentPixels) {
                cx += static_cast<float>(px);
                cy += static_cast<float>(py);
            }
            const float invCount = 1.0f / static_cast<float>(componentPixels.size());
            cx *= invCount;
            cy *= invCount;

            float maxDistSq = 0.0f;
            for (const auto& [px, py] : componentPixels) {
                const float dx = static_cast<float>(px) - cx;
                const float dy = static_cast<float>(py) - cy;
                maxDistSq = std::max(maxDistSq, dx * dx + dy * dy);
            }
            circles.push_back({Vec2{cx, cy}, std::max(2.0f, std::sqrt(maxDistSq))});
        }
    }

    return circles;
}

} // namespace simfw::simulation
