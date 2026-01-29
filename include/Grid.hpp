#pragma once

struct Vec2 {
    float x = 0.f;
    float y = 0.f;
};

struct IGrid {
    virtual ~IGrid() = default;

    virtual int width() const = 0;
    virtual int height() const = 0;

    // "Solid" means barrier/wall/ground tile (your previous isBarrier()).
    virtual bool isSolid(int x, int y) const = 0;
};
