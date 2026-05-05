#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <random/Random.hpp>

namespace sand_cpu {

namespace {
constexpr float kMaxSpawnSpeed = 1.5f;

float clampVelocity(float v, float limit) {
    return std::clamp(v, -limit, limit);
}

std::pair<int, int> inertialStep(const Cell& cell) {
    auto axisStep = [](float v) -> int {
        const float absV = std::fabs(v);
        if (absV < 0.05f) {
            return 0;
        }
        int step = 0;
        if (absV >= 1.0f) {
            step = 1;
        } else if (Random::range(0.0f, 1.0f) < absV) {
            step = 1;
        }
        return v < 0.0f ? -step : step;
    };

    return {axisStep(cell.vx), axisStep(cell.vy)};
}
}

Simulation::Simulation(SimulationConfig config) : Base(config) {
    m_config.entityCount = m_config.maxParticleCount;
    m_cells.resize(m_config.gridWidth * m_config.gridHeight);

    m_chunkCols = (m_config.gridWidth + m_config.chunkSize - 1) / m_config.chunkSize;
    m_chunkRows = (m_config.gridHeight + m_config.chunkSize - 1) / m_config.chunkSize;

    const std::size_t chunkCount = m_chunkCols * m_chunkRows;
    m_activeChunks.assign(chunkCount, 1);
    m_nextActiveChunks.assign(chunkCount, 0);

    m_stats.maxParticleCount = m_config.maxParticleCount;
    rebuildStats();
}

std::size_t Simulation::idx(int x, int y) const {
    return static_cast<std::size_t>(y) * m_config.gridWidth + static_cast<std::size_t>(x);
}

bool Simulation::inBounds(int x, int y) const {
    return x >= 0 && y >= 0 &&
           x < static_cast<int>(m_config.gridWidth) &&
           y < static_cast<int>(m_config.gridHeight);
}

void Simulation::markChunkDirtyByCell(int x, int y) {
    if (!inBounds(x, y)) {
        return;
    }

    const std::size_t cx = static_cast<std::size_t>(x) / m_config.chunkSize;
    const std::size_t cy = static_cast<std::size_t>(y) / m_config.chunkSize;
    const std::size_t chunkIndex = cy * m_chunkCols + cx;
    m_activeChunks[chunkIndex] = 1;
    m_nextActiveChunks[chunkIndex] = 1;
}

void Simulation::markNeighborsDirty(int x, int y) {
    for (int oy = -1; oy <= 1; ++oy) {
        for (int ox = -1; ox <= 1; ++ox) {
            markChunkDirtyByCell(x + ox, y + oy);
        }
    }
}

bool Simulation::tryMove(int x, int y, int nx, int ny, bool markActivity) {
    if (!inBounds(nx, ny) || !inBounds(x, y)) {
        return false;
    }

    Cell& from = m_cells[idx(x, y)];
    Cell& to = m_cells[idx(nx, ny)];

    if (from.material == Material::Empty || to.material != Material::Empty) {
        return false;
    }

    std::swap(from, to);

    if (markActivity) {
        markNeighborsDirty(x, y);
        markNeighborsDirty(nx, ny);
    }

    return true;
}

void Simulation::update(float) {
    m_stats.movedCells = 0;
    std::fill(m_nextActiveChunks.begin(), m_nextActiveChunks.end(), 0);

    for (int y = static_cast<int>(m_config.gridHeight) - 1; y >= 0; --y) {
        const bool leftToRight = (y & 1) == 0;

        for (int pass = 0; pass < static_cast<int>(m_config.gridWidth); ++pass) {
            const int x = leftToRight ? pass : static_cast<int>(m_config.gridWidth) - 1 - pass;

            const std::size_t cx = static_cast<std::size_t>(x) / m_config.chunkSize;
            const std::size_t cy = static_cast<std::size_t>(y) / m_config.chunkSize;
            if (m_activeChunks[cy * m_chunkCols + cx] == 0) {
                continue;
            }

            Cell& cell = m_cells[idx(x, y)];

            if (cell.material != Material::Empty) {
                const auto [inertialX, inertialY] = inertialStep(cell);
                if ((inertialX != 0 || inertialY != 0) && tryMove(x, y, x + inertialX, y + inertialY)) {
                    ++m_stats.movedCells;
                    Cell& movedCell = m_cells[idx(x + inertialX, y + inertialY)];
                    switch (movedCell.material) {
                        case Material::Sand:
                            movedCell.vy = clampVelocity(movedCell.vy + 0.18f, 1.6f);
                            movedCell.vx = clampVelocity(movedCell.vx * 0.86f, 1.2f);
                            break;
                        case Material::Water:
                            movedCell.vy = clampVelocity(movedCell.vy + 0.12f, 1.4f);
                            movedCell.vx = clampVelocity(movedCell.vx * 0.9f, 1.4f);
                            break;
                        case Material::Smoke:
                            movedCell.vy = clampVelocity(movedCell.vy - 0.08f, 1.0f);
                            movedCell.vx = clampVelocity(movedCell.vx * 0.92f, 1.2f);
                            break;
                        case Material::Empty:
                        default:
                            break;
                    }
                    continue;
                }
            }

            switch (cell.material) {
                case Material::Sand: {
                    bool moved = tryMove(x, y, x, y + 1);
                    if (!moved) {
                        const int dir = Random::range(0, 2) == 0 ? -1 : 1;
                        moved = tryMove(x, y, x + dir, y + 1) || tryMove(x, y, x - dir, y + 1);
                    }
                    if (moved) {
                        ++m_stats.movedCells;
                    }
                    cell.vy = clampVelocity(cell.vy + 0.18f, 1.6f);
                    cell.vx = clampVelocity(cell.vx * 0.86f, 1.2f);
                    break;
                }
                case Material::Water: {
                    bool moved = tryMove(x, y, x, y + 1);
                    if (!moved) {
                        const int dir = Random::range(0, 2) == 0 ? -1 : 1;
                        moved = tryMove(x, y, x + dir, y + 1) ||
                                tryMove(x, y, x - dir, y + 1) ||
                                tryMove(x, y, x + dir, y) ||
                                tryMove(x, y, x - dir, y);
                    }
                    if (moved) {
                        ++m_stats.movedCells;
                    }
                    cell.vy = clampVelocity(cell.vy + 0.12f, 1.4f);
                    cell.vx = clampVelocity(cell.vx * 0.9f, 1.4f);
                    break;
                }
                case Material::Smoke: {
                    bool moved = tryMove(x, y, x, y - 1);
                    if (!moved) {
                        const int dir = Random::range(0, 2) == 0 ? -1 : 1;
                        moved = tryMove(x, y, x + dir, y - 1) ||
                                tryMove(x, y, x - dir, y - 1) ||
                                tryMove(x, y, x + dir, y) ||
                                tryMove(x, y, x - dir, y);
                    }
                    if (moved) {
                        ++m_stats.movedCells;
                    } else if (cell.life > 0) {
                        --cell.life;
                        if (cell.life == 0) {
                            cell.material = Material::Empty;
                            markNeighborsDirty(x, y);
                        }
                    }
                    cell.vy = clampVelocity(cell.vy - 0.08f, 1.0f);
                    cell.vx = clampVelocity(cell.vx * 0.92f, 1.2f);
                    break;
                }
                case Material::Empty:
                default:
                    break;
            }
        }
    }

    m_activeChunks = m_nextActiveChunks;
    rebuildStats();
}

void Simulation::spawnDisc(int centerX, int centerY, Material material, float brushVx, float brushVy) {
    for (int i = 0; i < m_config.spawnAmount; ++i) {
        const int ox = Random::range(-m_config.brushRadius, m_config.brushRadius + 1);
        const int oy = Random::range(-m_config.brushRadius, m_config.brushRadius + 1);
        if (ox * ox + oy * oy > m_config.brushRadius * m_config.brushRadius) {
            continue;
        }

        const int x = centerX + ox;
        const int y = centerY + oy;
        if (!inBounds(x, y)) {
            continue;
        }

        Cell& cell = m_cells[idx(x, y)];
        if (cell.material == Material::Empty) {
            cell.material = material;
            cell.life = material == Material::Smoke ? static_cast<std::uint8_t>(Random::range(40, 120)) : 0;
            const float jitterX = Random::range(-0.25f, 0.25f);
            const float jitterY = Random::range(-0.25f, 0.25f);
            cell.vx = clampVelocity(brushVx * 0.65f + jitterX, kMaxSpawnSpeed);
            cell.vy = clampVelocity(brushVy * 0.65f + jitterY, kMaxSpawnSpeed);
            markNeighborsDirty(x, y);
        }
    }

    rebuildStats();
}

void Simulation::clear() {
    std::fill(m_cells.begin(), m_cells.end(), Cell{});
    std::fill(m_activeChunks.begin(), m_activeChunks.end(), 0);
    std::fill(m_nextActiveChunks.begin(), m_nextActiveChunks.end(), 0);
    rebuildStats();
}

void Simulation::reset() {
    clear();

    const int centerX = static_cast<int>(m_config.gridWidth / 2);
    for (int y = 10; y < 80; ++y) {
        for (int x = centerX - 30; x < centerX + 30; ++x) {
            if (!inBounds(x, y)) {
                continue;
            }
            Cell& cell = m_cells[idx(x, y)];
            cell.material = Material::Sand;
            markNeighborsDirty(x, y);
        }
    }

    rebuildStats();
}

void Simulation::rebuildStats() {
    std::size_t count = 0;
    for (const Cell& c : m_cells) {
        if (c.material != Material::Empty) {
            ++count;
        }
    }

    m_stats.particleCount = count;
    m_stats.entityCount = count;
    m_stats.maxParticleCount = m_config.maxParticleCount;
    m_stats.activeChunks = std::count(m_activeChunks.begin(), m_activeChunks.end(), static_cast<std::uint8_t>(1));
}

} // namespace sand_cpu
