#pragma once

namespace agents_cpu {

enum class AgentIntent {
    SeekTarget,
    AvoidObstacle,
    Idle
};

constexpr const char* intentName(AgentIntent intent) {
    switch (intent) {
    case AgentIntent::SeekTarget:
        return "seek target";
    case AgentIntent::AvoidObstacle:
        return "avoid obstacle";
    case AgentIntent::Idle:
        return "idle";
    }

    return "unknown";
}

constexpr bool isMovingIntent(AgentIntent intent) {
    return intent == AgentIntent::SeekTarget ||
        intent == AgentIntent::AvoidObstacle;
}

} // namespace agents_cpu
