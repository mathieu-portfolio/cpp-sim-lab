#pragma once

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <string>
#include <utility>

namespace bench {

class ProgressBar {
public:
    ProgressBar(std::size_t totalSteps, std::string label = "Benchmark progress", std::size_t width = 40)
        : m_totalSteps(std::max<std::size_t>(totalSteps, 1)), m_label(std::move(label)), m_width(width) {
        render();
    }

    void advance(std::size_t stepCount = 1) {
        m_completedSteps = std::min(m_completedSteps + stepCount, m_totalSteps);
        render();
    }

    void finish() {
        m_completedSteps = m_totalSteps;
        render();
        std::cerr << '\n';
    }

private:
    void render() const {
        const double ratio = static_cast<double>(m_completedSteps) / static_cast<double>(m_totalSteps);
        const std::size_t filled = static_cast<std::size_t>(ratio * static_cast<double>(m_width));

        std::cerr << '\r' << m_label << " [";
        for (std::size_t i = 0; i < m_width; ++i) {
            std::cerr << (i < filled ? '=' : ' ');
        }
        std::cerr << "] " << static_cast<int>(ratio * 100.0) << "% (" << m_completedSteps << '/' << m_totalSteps << ")" << std::flush;
    }

    std::size_t m_totalSteps;
    std::string m_label;
    std::size_t m_width;
    std::size_t m_completedSteps{0};
};

} // namespace bench
