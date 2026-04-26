#pragma once
#include <chrono>

class FixedTimestep {
public:
    explicit FixedTimestep(double dt) : m_dt(dt) {
        m_last = clock::now();
    }

    template<typename F>
    void update(F simulate) {
        auto now = clock::now();
        double frameTime = std::chrono::duration<double>(now - m_last).count();
        m_last = now;

        m_acc += frameTime;

        while (m_acc >= m_dt) {
            simulate(m_dt);
            m_acc -= m_dt;
        }
    }

private:
    using clock = std::chrono::high_resolution_clock;
    double m_dt;
    double m_acc = 0.0;
    std::chrono::time_point<clock> m_last;
};
