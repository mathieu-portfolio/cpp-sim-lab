#pragma once

namespace traffic_flow_cpu {

struct Vehicle {
    float s = 0.0f;
    float speed = 0.0f;
    float acceleration = 0.0f;
    int lane = 0;
    float length = 4.5f;
};

} // namespace traffic_flow_cpu
