#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace heat_grid_cpu {

Simulation::Simulation(SimulationConfig config) : Base(config) {
    m_config.entityCount = m_config.gridWidth * m_config.gridHeight;
    m_temperature.assign(m_config.entityCount, m_config.ambientTemperature);
    m_nextTemperature = m_temperature;
    m_sourceTemperature.assign(m_config.entityCount, 0.0f);
    seedFixedPoints();
    applyFixedPoints(m_temperature);
    applyFixedPoints(m_nextTemperature);
    rebuildStats();
}

std::size_t Simulation::idx(std::size_t x, std::size_t y) const {
    return y * m_config.gridWidth + x;
}

float Simulation::sample(int x, int y) const {
    const int w = static_cast<int>(m_config.gridWidth);
    const int h = static_cast<int>(m_config.gridHeight);

    if (m_config.boundaryMode == BoundaryMode::Wrap) {
        const int nx = (x % w + w) % w;
        const int ny = (y % h + h) % h;
        return m_temperature[idx(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny))];
    }

    if (m_config.boundaryMode == BoundaryMode::Clamp) {
        const int nx = std::clamp(x, 0, w - 1);
        const int ny = std::clamp(y, 0, h - 1);
        return m_temperature[idx(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny))];
    }

    if (m_config.boundaryMode == BoundaryMode::Insulated) {
        const int nx = (x < 0) ? 0 : ((x >= w) ? (w - 1) : x);
        const int ny = (y < 0) ? 0 : ((y >= h) ? (h - 1) : y);
        return m_temperature[idx(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny))];
    }

    if (x < 0 || y < 0 || x >= w || y >= h) {
        return m_config.ambientTemperature;
    }

    return m_temperature[idx(static_cast<std::size_t>(x), static_cast<std::size_t>(y))];
}

void Simulation::seedFixedPoints() {
    std::fill(m_sourceTemperature.begin(), m_sourceTemperature.end(), 0.0f);

    m_sourceTemperature[idx(m_config.gridWidth / 4, m_config.gridHeight / 2)] = 1.0f;
    m_sourceTemperature[idx((m_config.gridWidth * 3) / 4, m_config.gridHeight / 2)] = 0.85f;
    m_sourceTemperature[idx(m_config.gridWidth / 2, m_config.gridHeight / 3)] = -0.6f;
    m_sourceTemperature[idx(m_config.gridWidth / 2, (m_config.gridHeight * 2) / 3)] = -0.5f;
}

void Simulation::applyFixedPoints(std::vector<float>& field) {
    for (std::size_t i = 0; i < m_sourceTemperature.size(); ++i) {
        if (std::abs(m_sourceTemperature[i]) > 0.0001f) {
            field[i] = m_sourceTemperature[i];
        }
    }
}


void Simulation::adjustHeatSource(std::size_t x, std::size_t y, float delta) {
    if (x >= m_config.gridWidth || y >= m_config.gridHeight) {
        return;
    }

    const std::size_t sourceIndex = idx(x, y);
    m_sourceTemperature[sourceIndex] = std::clamp(m_sourceTemperature[sourceIndex] + delta, -1.0f, 1.0f);
    applyFixedPoints(m_temperature);
    applyFixedPoints(m_nextTemperature);
    rebuildStats();
}

void Simulation::clearHeatSource(std::size_t x, std::size_t y) {
    if (x >= m_config.gridWidth || y >= m_config.gridHeight) {
        return;
    }

    m_sourceTemperature[idx(x, y)] = 0.0f;
    applyFixedPoints(m_temperature);
    applyFixedPoints(m_nextTemperature);
    rebuildStats();
}

void Simulation::update(float) {
    for (std::size_t y = 0; y < m_config.gridHeight; ++y) {
        for (std::size_t x = 0; x < m_config.gridWidth; ++x) {
            const float c = sample(static_cast<int>(x), static_cast<int>(y));
            const float l = sample(static_cast<int>(x) - 1, static_cast<int>(y));
            const float r = sample(static_cast<int>(x) + 1, static_cast<int>(y));
            const float u = sample(static_cast<int>(x), static_cast<int>(y) - 1);
            const float d = sample(static_cast<int>(x), static_cast<int>(y) + 1);
            const float laplacian = (l + r + u + d) - (4.0f * c);

            const float sampleX = static_cast<float>(x) - m_config.advectionX;
            const float sampleY = static_cast<float>(y) - m_config.advectionY;
            const int x0 = static_cast<int>(std::floor(sampleX));
            const int y0 = static_cast<int>(std::floor(sampleY));
            const int x1 = x0 + 1;
            const int y1 = y0 + 1;
            const float tx = sampleX - static_cast<float>(x0);
            const float ty = sampleY - static_cast<float>(y0);

            const float t00 = sample(x0, y0);
            const float t10 = sample(x1, y0);
            const float t01 = sample(x0, y1);
            const float t11 = sample(x1, y1);
            const float advected =
                (1.0f - tx) * (1.0f - ty) * t00 +
                tx * (1.0f - ty) * t10 +
                (1.0f - tx) * ty * t01 +
                tx * ty * t11;

            m_nextTemperature[idx(x, y)] = advected + (m_config.diffusion * laplacian);
        }
    }

    applyFixedPoints(m_nextTemperature);
    std::swap(m_temperature, m_nextTemperature);
    rebuildStats();
}

void Simulation::clear() {
    std::fill(m_temperature.begin(), m_temperature.end(), m_config.ambientTemperature);
    std::fill(m_nextTemperature.begin(), m_nextTemperature.end(), m_config.ambientTemperature);
    applyFixedPoints(m_temperature);
    applyFixedPoints(m_nextTemperature);
    rebuildStats();
}

void Simulation::reset() {
    seedFixedPoints();
    clear();
}

void Simulation::rebuildStats() {
    float minT = std::numeric_limits<float>::max();
    float maxT = std::numeric_limits<float>::lowest();
    float sum = 0.0f;

    for (float t : m_temperature) {
        minT = std::min(minT, t);
        maxT = std::max(maxT, t);
        sum += t;
    }

    m_stats.minTemperature = minT;
    m_stats.maxTemperature = maxT;
    m_stats.avgTemperature = sum / static_cast<float>(m_temperature.size());
    m_stats.entityCount = m_temperature.size();
}

} // namespace heat_grid_cpu
