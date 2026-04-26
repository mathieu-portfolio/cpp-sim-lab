#pragma once
#include <random>

class Random {
public:
    static float range(float min, float max) {
        static thread_local std::mt19937 gen{std::random_device{}()};
        std::uniform_real_distribution<float> dist(min, max);
        return dist(gen);
    }
};
