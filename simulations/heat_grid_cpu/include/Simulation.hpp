#pragma once

#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>

#include <cstddef>
#include <vector>

namespace heat_grid_cpu {

enum class BoundaryMode {
    Clamp = 0,
    Wrap = 1,
    Insulated = 2,
};

struct HeatPoint {
    std::size_t x = 0;
    std::size_t y = 0;
    float temperature = 1.0f;
};

struct SimulationConfig {
    float width = 960.0f;
    float height = 720.0f;

    std::size_t gridWidth = 160;
    std::size_t gridHeight = 120;
    float cellSize = 6.0f;

    float diffusion = 0.2f;
    float ambientTemperature = 0.0f;
    BoundaryMode boundaryMode = BoundaryMode::Clamp;

    simfw::simulation::SimulationExecutionConfig execution{};
    std::size_t entityCount = 0;
};

struct SimulationStats {
    float minTemperature = 0.0f;
    float maxTemperature = 0.0f;
    float avgTemperature = 0.0f;
    std::size_t entityCount = 0;
};

class Simulation : public simfw::SimulationBase<SimulationConfig, SimulationStats, HeatPoint> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, HeatPoint>;

    explicit Simulation(SimulationConfig config = {});

    void update(float dt);
    void reset();
    void clear();

    const std::vector<float>& getTemperature() const { return m_temperature; }

    void addHeatSource(std::size_t x, std::size_t y, float temperature);
    void addHeatSink(std::size_t x, std::size_t y, float temperature);

private:
    std::vector<float> m_temperature;
    std::vector<float> m_nextTemperature;
    std::vector<HeatPoint> m_heatSources;
    std::vector<HeatPoint> m_heatSinks;

    [[nodiscard]] std::size_t idx(std::size_t x, std::size_t y) const;
    [[nodiscard]] float sample(int x, int y) const;
    void seedFixedPoints();
    void applyFixedPoints(std::vector<float>& field);
    void rebuildStats();
};

} // namespace heat_grid_cpu
