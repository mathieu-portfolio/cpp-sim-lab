#pragma once

#include <utility>
#include <vector>

namespace simfw {

template <
    typename ConfigT,
    typename StatsT,
    typename EntityT
>
class SimulationBase {
public:
    using Config = ConfigT;
    using Stats = StatsT;
    using Entity = EntityT;
    using EntityContainer = std::vector<Entity>;

    explicit SimulationBase(Config config = {})
        : m_config(std::move(config)) {}

    virtual ~SimulationBase() = default;

    SimulationBase(const SimulationBase&) = delete;
    SimulationBase& operator=(const SimulationBase&) = delete;

    SimulationBase(SimulationBase&&) = default;
    SimulationBase& operator=(SimulationBase&&) = default;

    Config& getConfig() {
        return m_config;
    }

    const Config& getConfig() const {
        return m_config;
    }

    Stats getStats() const {
        return m_stats;
    }

    const EntityContainer& getEntities() const {
        return m_entities;
    }

    EntityContainer& getEntities() {
        return m_entities;
    }

protected:
    EntityContainer m_entities;
    Config m_config;
    Stats m_stats;
};

} // namespace simfw
