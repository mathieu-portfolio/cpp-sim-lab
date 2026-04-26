#pragma once
#include <chrono>
#include <iostream>

class ScopedTimer {
public:
    ScopedTimer(const char* name) : m_name(name), m_start(clock::now()) {}
    ~ScopedTimer() {
        auto end = clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - m_start).count();
        std::cout << "[PROFILE] " << m_name << ": " << ms << " ms\n";
    }

private:
    using clock = std::chrono::high_resolution_clock;
    const char* m_name;
    std::chrono::time_point<clock> m_start;
};

#define SIM_PROFILE_SCOPE(name) ScopedTimer timer##__LINE__(name)
