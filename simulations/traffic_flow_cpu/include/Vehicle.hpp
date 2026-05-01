#pragma once

#include <cstddef>

namespace traffic_flow_cpu {

struct Vehicle {
    std::size_t roadId = 0;
    int laneId = 0;
    float s = 0.0f;
    float speed = 0.0f;
    float acceleration = 0.0f;
    float length = 4.5f;
};

} // namespace traffic_flow_cpu
