#pragma once

#include <raylib.h>
#include <simulation/ObstacleMask.hpp>

#include <cstdint>
#include <string>

namespace simfw::ui {

inline bool isBlockedPixel(const Color& pixel) {
    return pixel.r == 0 && pixel.g == 0 && pixel.b == 0;
}

inline bool isWalkablePixel(const Color& pixel) {
    return pixel.r == 255 && pixel.g == 255 && pixel.b == 255;
}

inline bool loadObstacleMaskFromPng(
    const std::string& filePath,
    simfw::simulation::ObstacleMask& mask,
    bool strictBinary = false
) {
    Image image = LoadImage(filePath.c_str());
    if (!IsImageValid(image)) {
        return false;
    }
    if (image.width <= 0 || image.height <= 0) {
        UnloadImage(image);
        return false;
    }

    ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    Color* pixels = LoadImageColors(image);
    if (pixels == nullptr) {
        UnloadImage(image);
        return false;
    }

    mask.resize(image.width, image.height);
    bool ok = true;
    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            const Color pixel = pixels[y * image.width + x];
            bool blocked = false;
            if (isBlockedPixel(pixel)) {
                blocked = true;
            } else if (isWalkablePixel(pixel)) {
                blocked = false;
            } else if (strictBinary) {
                ok = false;
            } else {
                const int luma = (299 * static_cast<int>(pixel.r) + 587 * static_cast<int>(pixel.g) + 114 * static_cast<int>(pixel.b)) / 1000;
                blocked = luma < 128;
            }
            mask.setBlocked(x, y, blocked);
        }
    }

    UnloadImageColors(pixels);
    UnloadImage(image);
    return ok;
}

inline bool exportObstacleMaskToPng(const std::string& filePath, const simfw::simulation::ObstacleMask& mask) {
    if (mask.width() <= 0 || mask.height() <= 0) {
        return false;
    }

    Image image = GenImageColor(mask.width(), mask.height(), WHITE);
    if (!IsImageValid(image)) {
        return false;
    }

    ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    Color* pixels = LoadImageColors(image);
    if (pixels == nullptr) {
        UnloadImage(image);
        return false;
    }

    for (int y = 0; y < mask.height(); ++y) {
        for (int x = 0; x < mask.width(); ++x) {
            pixels[y * mask.width() + x] = mask.isBlocked(x, y) ? BLACK : WHITE;
        }
    }

    Image out = {pixels, mask.width(), mask.height(), 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8};
    const bool ok = ExportImage(out, filePath.c_str());
    UnloadImageColors(pixels);
    UnloadImage(image);
    return ok;
}

} // namespace simfw::ui
