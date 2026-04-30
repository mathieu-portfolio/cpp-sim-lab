#pragma once

#include <math/Vec2.hpp>
#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace simfw::ui {

inline bool isObstacleMaskPixel(const Color& pixel, uint8_t threshold) {
    const float luma = 0.299f * static_cast<float>(pixel.r) +
        0.587f * static_cast<float>(pixel.g) +
        0.114f * static_cast<float>(pixel.b);
    return luma >= static_cast<float>(threshold);
}

inline bool loadObstacleShapesFromPng(
    const std::string& filePath,
    uint8_t threshold,
    const std::function<void()>& clearObstacles,
    const std::function<void(Vec2, float)>& addObstacle
) {
    Image image = LoadImage(filePath.c_str());
    if (!IsImageValid(image)) {
        return false;
    }

    const int width = image.width;
    const int height = image.height;
    if (width <= 0 || height <= 0) {
        UnloadImage(image);
        return false;
    }

    ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    Color* pixels = LoadImageColors(image);
    if (pixels == nullptr) {
        UnloadImage(image);
        return false;
    }

    std::vector<uint8_t> visited(static_cast<std::size_t>(width * height), 0u);
    auto indexOf = [width](int x, int y) {
        return static_cast<std::size_t>(y * width + x);
    };

    clearObstacles();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t startIndex = indexOf(x, y);
            if (visited[startIndex] != 0u || !isObstacleMaskPixel(pixels[startIndex], threshold)) {
                continue;
            }

            std::queue<std::pair<int, int>> queue;
            std::vector<std::pair<int, int>> componentPixels;
            queue.push({x, y});
            visited[startIndex] = 1u;

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
                        if (visited[nIndex] != 0u ||
                            !isObstacleMaskPixel(pixels[nIndex], threshold)) {
                            continue;
                        }
                        visited[nIndex] = 1u;
                        queue.push({nx, ny});
                    }
                }
            }

            if (componentPixels.empty()) {
                continue;
            }

            float centroidX = 0.0f;
            float centroidY = 0.0f;
            for (const auto& [px, py] : componentPixels) {
                centroidX += static_cast<float>(px);
                centroidY += static_cast<float>(py);
            }

            const float invCount = 1.0f / static_cast<float>(componentPixels.size());
            centroidX *= invCount;
            centroidY *= invCount;

            float maxDistSq = 0.0f;
            for (const auto& [px, py] : componentPixels) {
                const float dx = static_cast<float>(px) - centroidX;
                const float dy = static_cast<float>(py) - centroidY;
                maxDistSq = std::max(maxDistSq, dx * dx + dy * dy);
            }

            addObstacle(
                Vec2{centroidX, centroidY},
                std::max(2.0f, std::sqrt(maxDistSq))
            );
        }
    }

    UnloadImageColors(pixels);
    UnloadImage(image);
    return true;
}

inline bool exportObstacleShapesToPng(
    const std::string& filePath,
    int imageWidth,
    int imageHeight,
    const std::vector<std::pair<Vec2, float>>& circles
) {
    Image image = GenImageColor(imageWidth, imageHeight, BLACK);
    if (!IsImageValid(image)) {
        return false;
    }

    for (const auto& [position, radius] : circles) {
        ImageDrawCircleV(
            &image,
            Vector2{position.x, position.y},
            static_cast<int>(radius),
            WHITE
        );
    }

    const bool ok = ExportImage(image, filePath.c_str());
    UnloadImage(image);
    return ok;
}

inline bool paintObstacleMaskCircle(
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

inline std::vector<std::pair<Vec2, float>> buildObstacleCirclesFromMask(
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

} // namespace simfw::ui
