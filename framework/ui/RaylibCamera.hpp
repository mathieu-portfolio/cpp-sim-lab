#pragma once

#include <algorithm>

#include <raylib.h>

namespace simfw::ui {

inline Camera2D makeCenteredCamera(
    float worldWidth,
    float worldHeight,
    float zoom = 1.0f
) {
    Camera2D camera{};
    camera.target = Vector2{worldWidth * 0.5f, worldHeight * 0.5f};
    camera.offset = Vector2{worldWidth * 0.5f, worldHeight * 0.5f};
    camera.rotation = 0.0f;
    camera.zoom = zoom;

    return camera;
}

inline void clampCameraToBounds(
    Camera2D& camera,
    float worldWidth,
    float worldHeight
) {
    const float viewWidth = worldWidth / camera.zoom;
    const float viewHeight = worldHeight / camera.zoom;

    if (viewWidth >= worldWidth) {
        camera.target.x = worldWidth * 0.5f;
    } else {
        const float halfViewWidth = viewWidth * 0.5f;
        camera.target.x = std::clamp(
            camera.target.x,
            halfViewWidth,
            worldWidth - halfViewWidth
        );
    }

    if (viewHeight >= worldHeight) {
        camera.target.y = worldHeight * 0.5f;
    } else {
        const float halfViewHeight = viewHeight * 0.5f;
        camera.target.y = std::clamp(
            camera.target.y,
            halfViewHeight,
            worldHeight - halfViewHeight
        );
    }
}

inline void resetCameraToBounds(
    Camera2D& camera,
    float worldWidth,
    float worldHeight,
    float zoom = 1.0f
) {
    camera = makeCenteredCamera(worldWidth, worldHeight, zoom);
}

inline void zoomCameraAtScreenPoint(
    Camera2D& camera,
    Vector2 screenPoint,
    float wheelDelta,
    float zoomStep,
    float minZoom,
    float maxZoom,
    float worldWidth,
    float worldHeight
) {
    if (wheelDelta == 0.0f) {
        return;
    }

    const Vector2 worldBefore = GetScreenToWorld2D(screenPoint, camera);

    camera.zoom *= 1.0f + wheelDelta * zoomStep;
    camera.zoom = std::clamp(camera.zoom, minZoom, maxZoom);

    const Vector2 worldAfter = GetScreenToWorld2D(screenPoint, camera);

    camera.target.x += worldBefore.x - worldAfter.x;
    camera.target.y += worldBefore.y - worldAfter.y;

    clampCameraToBounds(camera, worldWidth, worldHeight);
}

inline void panCameraByScreenDelta(
    Camera2D& camera,
    Vector2 screenDelta,
    float worldWidth,
    float worldHeight
) {
    camera.target.x -= screenDelta.x / camera.zoom;
    camera.target.y -= screenDelta.y / camera.zoom;

    clampCameraToBounds(camera, worldWidth, worldHeight);
}

} // namespace simfw::ui
