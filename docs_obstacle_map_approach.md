# Obstacle Scenario PNG Import/Export Approach (Shape-Based)

## Short answer

Yes — it's feasible to use PNGs with **drawn obstacle shapes** (not just per-cell occupancy).

The robust approach is:

1. Treat the PNG as a **binary mask** of obstacle regions.
2. Extract **connected components** and their **contours**.
3. Convert contours into simulation geometry (polygons, circles, capsules, or simplified segments).
4. Keep an optional occupancy/grid cache for fast broad-phase queries.

## Why shape-based PNG works

A raster image still gives you exact obstacle regions in pixel space. From that raster mask, you can recover vector-like geometry with standard image-processing steps.

So instead of: "one white pixel = one obstacle cell", you do: "one white region = one obstacle shape".

## Recommended pipeline

### 1) Load and binarize

- Decode PNG (`stb_image` or `lodepng`), convert to grayscale/luma.
- Threshold to produce a binary obstacle mask.
- Optional invert flag.

### 2) Clean the mask

- Optional morphological open/close to remove noise and tiny holes.
- Minimum area filter to drop specks.

### 3) Extract obstacle regions

- Run connected-component labeling (4- or 8-connectivity).
- For each component, trace outer contour (and inner holes if needed).

### 4) Simplify geometry

For each contour:

- Simplify polyline (Ramer-Douglas-Peucker).
- Enforce clockwise/counter-clockwise winding.
- Convert from pixel coordinates to world coordinates.

### 5) Build runtime collision representation

Depending on your current collision model, choose one:

- **Polygon obstacles** (most faithful).
- **Segment chains / edges** (good for steering and ray tests).
- **Primitive decomposition** (circles/rectangles) for faster checks.

### 6) Export

Two export modes are useful:

- **Mask export**: render current obstacle geometry back to PNG.
- **Geometry export**: save polygons/primitives in JSON.

Using both lets artists edit masks while runtime keeps stable geometry.

## Practical format suggestion

Use PNG + JSON sidecar:

```json
{
  "image": "warehouse_shapes.png",
  "threshold": 128,
  "invert": false,
  "pixelToWorld": 4.0,
  "origin": [0.0, 0.0],
  "connectivity": 8,
  "minComponentAreaPx": 12,
  "simplifyEpsilonPx": 1.5,
  "mode": "contour"
}
```

`mode` can be:

- `mask_cells` (old cell-per-pixel approach),
- `contour` (shape extraction),
- `hybrid` (contours + cached occupancy).

## C++ API shape

```cpp
struct BinaryMask {
    int width;
    int height;
    std::vector<uint8_t> blocked; // 0/1 per pixel
};

struct PolygonObstacle {
    std::vector<Vec2> vertices;
};

struct ObstacleScene {
    std::vector<PolygonObstacle> polygons;
};

BinaryMask loadMaskFromPng(const std::string& path, uint8_t threshold, bool invert);
ObstacleScene extractObstacleScene(const BinaryMask& mask,
                                   float pixelToWorld,
                                   Vec2 origin,
                                   int connectivity,
                                   int minAreaPx,
                                   float simplifyEpsilonPx);
void saveMaskToPng(const std::string& path, const BinaryMask& mask);
```

## Tradeoffs vs pure cell map

- **Pros**
  - Natural authoring: users can paint/draw freeform shapes.
  - Fewer runtime objects than one-obstacle-per-cell.
  - Better visual fidelity.
- **Cons**
  - More preprocessing complexity (contours/simplification).
  - Extra care needed for thin features and anti-aliased edges.

## Important implementation details

- Disable anti-aliased brush when authoring masks, or use robust thresholding.
- Decide if diagonal-touching pixels are same obstacle (8-connectivity) or separate (4-connectivity).
- Handle holes explicitly if your simulation should let agents pass through "donut" centers.
- Clamp/snap contour vertices to avoid tiny self-intersections after simplification.

## Best approach for your repo right now

Given your current simulations are obstacle-primitive based, the best incremental path is:

1. Implement `contour` extraction from PNG.
2. Approximate each contour as either:
   - polygon edges (new obstacle type), or
   - merged circles/rectangles (if you want to stay with current simple checks first).
3. Also generate a coarse occupancy cache for broad-phase candidate lookup.

That gives shape-based authoring now, and you can keep optimizing runtime representation later.
