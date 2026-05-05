#pragma once

#include <cstddef>

namespace traffic_flow {

struct Vehicle {
    std::size_t roadId = 0;
    int laneId = 0;
    float s = 0.0f;
    float speed = 0.0f;
    float acceleration = 0.0f;
    float length = 4.5f;
    float crossroadWaitTime = 0.0f;
    float crossroadClearTime = 0.0f;
    float crossroadReleaseTime = 0.0f;
    float lastMovingRightPriorityTime = -1000000.0f;
    int reservedCrossroadId = -1;
    bool crossroadEngaged = false;
};

} // namespace traffic_flow
