#pragma once

#include <cstdint>
#include <random>

class Random {
public:
    static void seed(std::uint32_t value) {
        engine().seed(value);
    }

    static void seedFromRandomDevice() {
        engine().seed(std::random_device{}());
    }

    static float range(float min, float max) {
        std::uniform_real_distribution<float> dist(min, max);
        return dist(engine());
    }

private:
    static std::mt19937& engine() {
        static thread_local std::mt19937 gen{std::random_device{}()};
        return gen;
    }
};
